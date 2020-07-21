// Copyright 2018 Mipmap Games.

#include "RealisticProjectileComponent.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "Runtime/Engine/Classes/Components/PrimitiveComponent.h"
#include "Runtime/Engine/Classes/Components/StaticMeshComponent.h"
#include "Runtime/Engine/Classes/Engine/Engine.h"
#include "Runtime/Engine/Classes/GameFramework/PlayerController.h"
#include "Runtime/Launch/Resources/Version.h" // ENGINE_MINOR_VERSION

URealisticProjectileComponent::URealisticProjectileComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	static ConstructorHelpers::FObjectFinder<UDataTable> MatProps(*DataTablePath);
	MaterialPropertiesTable = MatProps.Object;

	Velocity = FVector(1.f, 0.f, 0.f);

	TempHitResults.Reserve(32); // 32 seems like a good amount of memory to reserve

	bWantsInitializeComponent = true;
}

void URealisticProjectileComponent::InitializeComponent()
{
	Super::InitializeComponent();

	Velocity = Velocity.GetSafeNormal() * Settings.InitialSpeed;
	if (Velocity.SizeSquared() < SMALL_NUMBER && Settings.InitialSpeed > SMALL_NUMBER) { Velocity = FVector(Settings.InitialSpeed, 0, 0); }
	SetVelocityInLocalSpace(Velocity);

	/*if (bInitialVelocityInLocalSpace)
	{
		SetVelocityInLocalSpace(Velocity);
	}*/
	if (UpdatedComponent)
	{
		if (Settings.bRotationFollowsVelocity)
		{
			UpdatedComponent->SetWorldRotation(Velocity.Rotation());
		}
	}

	UpdateComponentVelocity();

	if (UpdatedPrimitive && UpdatedPrimitive->IsSimulatingPhysics())
	{
		UpdatedPrimitive->SetPhysicsLinearVelocity(Velocity);
	}

	// Set up trajectory initial conditions
	FVector Loc0 = FVector::ZeroVector;
	AActor* ActorOwner = UpdatedComponent->GetOwner();
	FRotator InitialRotation = FRotator::ZeroRotator;
	if (IsValid(ActorOwner))
	{
		Loc0 = ActorOwner->GetActorLocation();
		InitialRotation = ActorOwner->GetActorRotation();
	}
	
	MoveEntryHits.SetNum(32, false);

	float pitch = FMath::FRandRange(AngularVelocityMin.Pitch, AngularVelocityMax.Pitch);
	float yaw = FMath::FRandRange(AngularVelocityMin.Yaw, AngularVelocityMax.Yaw);
	float roll = FMath::FRandRange(AngularVelocityMin.Roll, AngularVelocityMax.Roll);
	AngularVelocity = FRotator(pitch, yaw, roll);

	const UWorld* MyWorld = GetWorld();
	if (!IsValid(MyWorld))
	{
		InitialConditions = FTrajectoryInitialConditions(Velocity, Loc0, 0.f, InitialRotation);
	}
	else
	{
		InitialConditions = FTrajectoryInitialConditions(Velocity, Loc0, MyWorld->GetTimeSeconds(), InitialRotation);
	}
	InitialConditionHistory.Emplace(InitialConditions);
}

void URealisticProjectileComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
	QUICK_SCOPE_CYCLE_COUNTER(STAT_ProjectileMovementComponent_TickComponent);
	// skip if don't want component updated when not rendered or updated component can't move
	if (HasStoppedSimulation())
	{
		return;
	}
	if (ShouldSkipUpdate(DeltaTime)) return;

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!IsValid(UpdatedComponent))
	{
		return;
	}

	AActor* ActorOwner = UpdatedComponent->GetOwner();
	if (!ActorOwner || !CheckStillInWorld())
	{
		return;
	}

	if (UpdatedComponent->IsSimulatingPhysics())
	{
		return;
	}

	const UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	float TickRemainingTime = DeltaTime;
	int32 Iterations = 0;

	k = fabsf(Settings.TerminalVelocity) < SMALL_NUMBER ? 9.f : fabsf(0.5f * GetGravityZ() / Settings.TerminalVelocity);
	Vinf = Settings.Gravity.GetSafeNormal() * Settings.TerminalVelocity;

	const float GameTime = World->GetTimeSeconds();

	while (TickRemainingTime > 0.f && !HasStoppedSimulation() && Iterations++ < 500)
	{
		struct SubTickState BeforeTick = SubTickState(UpdatedComponent->GetComponentLocation(), UpdatedComponent->GetComponentRotation(), Velocity, InitialConditions, TickRemainingTime);
		struct SubTickState AfterTick;
		InternalSubTick(this, AfterTick, BeforeTick, World, GameTime);			
#if WITH_EDITOR
		DoDrawDebugLine(BeforeTick.Location, AfterTick.Location, BeforeTick.Velocity.Size());
#endif
		AfterTick.SetFromState(this, TickRemainingTime);

		if (TickRemainingTime >= DeltaTime - SMALL_NUMBER)
		{
			//UE_LOG(LogTemp, Warning, TEXT("Forgot to change tick remaining time? %d subticks. MoveType=%d"), Iterations, (int32)BeforeTick.Trajectory.TrajectoryType)
		}

		if (HasStoppedMoving(this)) {
			if (InitialConditions.TrajectoryType == FTrajectoryInitialConditions::ETrajectoryType::TT_In_Object) { TryEmbed(); }
			FHitResult LastHit = ObjectsPenetrated.Num() > 0 ? ObjectsPenetrated[0] : FHitResult();
			if (UpdatedComponent != nullptr) { StopSimulating(LastHit); }
		}
	}
}

void URealisticProjectileComponent::InternalSubTick(URealisticProjectileComponent* self, struct SubTickState& OutState, const struct SubTickState& InState, const UWorld* World, float GameTime, bool PredictOnly)
{
	if (self == nullptr || self->UpdatedPrimitive == nullptr) { OutState.TickRemainingTime = 0.f; return; }
	//UE_LOG(ProjectilePhysics, Display, TEXT("Subtick t=%f t0=%f Inside %d objs spd=%f Loc=%f,%f dt=%f"), 
	//	GameTime, self->InitialConditions.t0, self->ObjectsPenetrated.Num(), self->Velocity.Size(), InState.Location.X, InState.Location.Y, InState.TickRemainingTime)
		
	float TimeSinceLaunch = GameTime - InState.Trajectory.t0;
	if (self->InitialConditions.TrajectoryType == FTrajectoryInitialConditions::ETrajectoryType::TT_In_Object && TimeSinceLaunch > InState.Trajectory.EndTrajectoryTime)
	{
		// Set dt to the rest-time so newlocation is the furthest it will go
		TimeSinceLaunch = InState.Trajectory.EndTrajectoryTime;
	}

	FVector NewLocation = self->ComputeNewLocation(TimeSinceLaunch);
	FRotator NewRotation = self->ComputeNewRotation(TimeSinceLaunch);
	FVector NewVelocity = self->ComputeNewVelocity(TimeSinceLaunch);
		
	self->MoveEntryHits.Reset();
	self->DoSweep(InState.Location, NewLocation, InState.Rotation, self->MoveEntryHits, World, true);

	if (PredictOnly)
	{
		self->MoveEntryHits.RemoveAll([self](const FHitResult& H)
		{
			return self->UpdatedPrimitive->GetCollisionResponseToComponent(H.GetComponent()) <= ECollisionResponse::ECR_Overlap;
		});
	}
	else
	{
		// Check for overlaps IE. hit a player pawn.
		// First remove things we exited since last tick
		for (int Index = self->OverlappedComponents.Num() - 1; Index >= 0; Index--)
		{
			UPrimitiveComponent* PC = self->OverlappedComponents[Index];
			if (!self->MoveEntryHits.ContainsByPredicate([PC](const FHitResult& Hit) {return Hit.GetComponent() == PC; }))
			{
				self->NotifyHitObjectOfEndOverlap(PC);
				self->OverlappedComponents.RemoveAt(Index);
			}
		}
		// Add things we just overlapped and handle new overlaps with broadcast
		for (int32 Index = self->MoveEntryHits.Num() - 1; Index >= 0; --Index)
		{
			FHitResult CurrentHit = self->MoveEntryHits[Index];
			UPrimitiveComponent* OtherComponent = CurrentHit.GetComponent();
			ECollisionResponse CR = self->UpdatedPrimitive->GetCollisionResponseToComponent(OtherComponent);
			if (CR < ECollisionResponse::ECR_Block) {
				if (!self->OverlappedComponents.Contains(OtherComponent))
				{
					self->OverlappedComponents.Emplace(CurrentHit.GetComponent());
					self->BroadcastHitOrOverlap(CurrentHit);
					self->ApplyImpulse(self->MoveEntryHits[Index], NewVelocity);
				}
				self->MoveEntryHits.RemoveAt(Index);
			}
		}
	}

	// Now remove all hits for objects we are already inside.
	self->MoveEntryHits.RemoveAll([self](const FHitResult& Hit) 
	{
		return self->ObjectsPenetrated.ContainsByPredicate([&Hit](const FHitResult& AlreadyInside)
		{
			return Hit.GetComponent() == AlreadyInside.GetComponent();
		});
	});
	// Now handle hits
	struct SubTickState StateAfterTravel = SubTickState(NewLocation, NewRotation, NewVelocity, InState.Trajectory, 0.f);
	if (self->InitialConditions.TrajectoryType == FTrajectoryInitialConditions::ETrajectoryType::TT_In_Object) // We are inside another object at the start of the tick
	{
		
		self->HandleInObjectTick(OutState, StateAfterTravel, GameTime, World, PredictOnly);
	}
	else // We are in air (or sliding which is handled the same)
	{
		self->HandleAirTick(OutState, StateAfterTravel, InState.Location, TimeSinceLaunch, GameTime, World, PredictOnly);
	}
}

FVector URealisticProjectileComponent::ComputeNewLocation(float TimeSinceLaunch)
{
	float DeltaTime = TimeSinceLaunch;
	FTrajectoryInitialConditions TIC = InitialConditions; // for shortness
	switch (TIC.TrajectoryType)
	{
	case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Air:
	{
		//UE_LOG(ProjectilePhysics, Display, TEXT("Calculating new air location: t0=%f V=%f v0=%f dt=%f Y=%f"),
		//	InitialConditions.t0, Velocity.Size(), InitialConditions.v0length, TimeSinceLaunch, UpdatedComponent->GetComponentLocation().Y)
			return URealisticProjectileComponent::PositionAtTime_LinearDrag(TIC, k, Vinf, DeltaTime);
	}
	case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Object:
	{
		// x(t) = 0.5at^2 + vt + x0
		FVector xt = (0.5f * TIC.PenDeceleration * DeltaTime * DeltaTime * TIC.v0direction) +
			(TIC.InitialVelocity * DeltaTime);
		return xt + TIC.InitialWorldLocation;
	}
	case FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding:
	{
		if (Settings.Friction <= 0.f)
		{
			// Sliding - deterministic but frictionless
			FVector xt = 0.5f * TIC.SlidingAcceleration * DeltaTime * DeltaTime + TIC.InitialVelocity * DeltaTime;
			return xt + TIC.InitialWorldLocation;
		}
		else
		{
			// Friction is done piecewise so we need time since last tick. Since the whole class is set up to use initial conditions,
			// probably the least impact thing to do is remember the details from last tick
			if (InitialConditions.t0 != FrLastLocationComputeTrajectory)
			{
				FrLastLocationComputeTime = 0.f;
				FrLastLocationComputeTrajectory = InitialConditions.t0;
			}

			float TimeIncrement = TimeSinceLaunch - FrLastLocationComputeTime;
			// If asked for new location for the same time, do nothing.
			if (fabsf(TimeIncrement) < SMALL_NUMBER || UpdatedPrimitive == nullptr)
			{
				return UpdatedPrimitive == nullptr ? FVector::ZeroVector : UpdatedPrimitive->GetComponentLocation();
			}
			FVector xt = 0.5f * TIC.SlidingAcceleration * TimeIncrement * TimeIncrement + Velocity * TimeIncrement;
			FVector FrictionComponent = 0.5f * Settings.Friction * GetGravityZ() * xt.GetSafeNormal() * TimeIncrement * TimeIncrement;
			// Sometimes we'll want to walk back the projectile.
			if (TimeIncrement < 0.f) { xt -= FrictionComponent; }
			else
			{
				// Friction can't make the object move backwards
				if (FrictionComponent.SizeSquared() > xt.SizeSquared()) { FrictionComponent = xt; }
				xt += FrictionComponent;
			}
			FrLastLocationComputeTime = TimeSinceLaunch;
			return UpdatedPrimitive->GetComponentLocation() + xt;
		}
	}
	default:
		return FVector::ZeroVector;
	}
}

FRotator URealisticProjectileComponent::ComputeNewRotation(float TimeSinceLaunch)
{
	float DeltaTime = TimeSinceLaunch;
	switch (InitialConditions.TrajectoryType)
	{
	case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Air:
		// Fall through: calculation is the same
	case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Object:
	{
		if (Settings.bRotationFollowsVelocity)
		{
			float Roll = InitialConditions.InitialRotation.Roll + AngularVelocity.Roll * DeltaTime;
			return Velocity.ToOrientationRotator() + FRotator(0, 0, Roll);
		}
		else
		{
			return InitialConditions.InitialRotation + AngularVelocity * DeltaTime;
		}
	}
	case FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding:
	{
		// Update rotation to appear rolling
		if (Settings.bRollsWhenSliding && IsValid(UpdatedComponent))
		{
			if (Settings.Friction <= 0.f)
			{
				float Radius = UpdatedComponent->Bounds.GetSphere().W;
				FVector Distance = ComputeNewLocation(TimeSinceLaunch) - UpdatedComponent->GetComponentLocation();
				FVector RotAxis = FVector::CrossProduct(Distance, Distance - InitialConditions.SlidingSurfaceNormal);
				float theta = Distance.Size() / Radius;
				FQuat dr = FQuat(RotAxis.GetUnsafeNormal(), fmodf(theta, 2 * PI)); // delta rotation
				UpdatedPrimitive->AddWorldRotation(dr, false);
				return UpdatedComponent->GetComponentRotation();
			}
			else
			{
				// Friction is done piecewise. Since the whole class is set up to use initial conditions,
				// the easiest thing to do is remember the last game time this function was asked to calculate sliding rot
				if (InitialConditions.t0 != FrLastRotationComputeTrajectory)
				{
					FrLastRotationComputeTime = 0.f;
					FrLastRotationComputeTrajectory = InitialConditions.t0;
				}
				float TimeIncrement = TimeSinceLaunch - FrLastRotationComputeTime;
				if (TimeIncrement < SMALL_NUMBER || UpdatedComponent == nullptr)
				{
					return UpdatedComponent == nullptr ? FRotator::ZeroRotator : UpdatedComponent->GetComponentRotation();
				}
				float Radius = UpdatedComponent->Bounds.GetSphere().W;
				FVector Distance = Velocity * TimeIncrement; // An approximation should be good enough
				FVector RotAxis = FVector::CrossProduct(Distance, Distance - InitialConditions.SlidingSurfaceNormal);
				float theta = Distance.Size() / Radius;
				FQuat dr = FQuat(RotAxis.GetUnsafeNormal(), fmodf(theta, 2 * PI)); // delta rotation
				UpdatedPrimitive->AddWorldRotation(dr, false);
				FrLastRotationComputeTime = TimeSinceLaunch;
				return UpdatedComponent->GetComponentRotation();
			}
		}
	}
	default:
		return FRotator::ZeroRotator;
	}
}

FVector URealisticProjectileComponent::ComputeNewVelocity(float TimeSinceLaunch)
{
	float DeltaTime = TimeSinceLaunch;
	switch (InitialConditions.TrajectoryType)
	{
	case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Air:
	{
		return URealisticProjectileComponent::VelocityAtTime_LinearDrag(InitialConditions, k, Vinf, DeltaTime);
	}
	case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Object:
	{
		FVector dv = InitialConditions.PenDeceleration * DeltaTime * InitialConditions.v0direction;
		return InitialConditions.InitialVelocity + dv;
	}
	case FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding:
	{
		if (Settings.Friction <= 0.f)
		{
			// V(t) = at + v0
			return InitialConditions.SlidingAcceleration * DeltaTime + InitialConditions.InitialVelocity;
		}
		else
		{
			// Friction is done piecewise. Since the whole class is set up to use initial conditions,
			// the easiest thing to do is remember the last game time this function was asked to calculate sliding vel
			if (InitialConditions.t0 != FrLastVelocityComputeTrajectory)
			{
				FrLastVelocityComputeTime = 0.f;
				FrLastVelocityComputeTrajectory = InitialConditions.t0;
				FrGravityComponentSizeSquared = InitialConditions.SlidingAcceleration.SizeSquared(); // save doing a sizesq every tick
			}
			float TimeIncrement = TimeSinceLaunch - FrLastVelocityComputeTime;
			if (fabsf(TimeIncrement) < SMALL_NUMBER)
			{
				return Velocity;
			}
			float FrictionForce = Settings.Friction * GetGravityZ();
			FVector FrictionComponent = FrictionForce * Velocity.GetSafeNormal() * TimeIncrement;
			FVector GravityComponent = InitialConditions.SlidingAcceleration * TimeIncrement;
			FVector NewVelocity = Velocity + FrictionComponent + GravityComponent;
			//UE_LOG(ProjectilePhysics, Display, TEXT("Compute vel with friction old=%f, new=%f, same direction=%d lastquery=%f thisquery=%f"),
			//	Velocity.Size(), NewVelocity.Size(), (NewVelocity | Velocity) > 0.f, FrLastVelocityComputeTime, TimeSinceLaunch)
			// If friction overcame accel and velocity
			if (FrictionForce * FrictionForce > FrGravityComponentSizeSquared && // Friction can overcome accel
				(NewVelocity | GravityComponent) < 0.f && // Velocity is against gravity after
				((Velocity | GravityComponent) > 0.f || Velocity.IsNearlyZero())) // Velocity was with gravity before (don't stop uphill) or 0
			{
				NewVelocity = FVector::ZeroVector;
			}
			FrLastVelocityComputeTime = TimeSinceLaunch;
			return NewVelocity;
		}
	}
	default:
		return FVector::ZeroVector;
	}
}

void URealisticProjectileComponent::DrawPredictedPath(UObject* WorldContextObject, TArray<UStaticMeshComponent*> Meshes, FVector StartLocation, FRotator StartRotation,
		TSubclassOf<AActor> ProjectileClass, int32 RandSeed, float TimeToSimulate, float StepTime, bool bOverrideGravity, FVector GravityAccel, float Thickness)
{
	if (Meshes.Num() == 0 || Meshes[0] == nullptr || WorldContextObject == nullptr || GEngine == nullptr) { return; }

	// It helps to scale meshes wider as they get futher away or you can't see them. So we'll need the camera location
	FVector CameraLocation;
	FRotator CameraRotation;
	APlayerController* Viewer = GEngine->GetFirstLocalPlayerController(GEngine->GetWorldFromContextObjectChecked(WorldContextObject));
	if (Viewer) { Viewer->GetPlayerViewPoint(CameraLocation, CameraRotation); }
	else { CameraLocation = StartLocation; }

	TArray<FVector> Locations;
	FVector Dummy;
	URealisticProjectileComponent::GetPredictedTrajectory(WorldContextObject, Locations, Dummy, StartLocation, StartRotation, ProjectileClass, RandSeed, TimeToSimulate, 
		StepTime, Meshes.Num(), bOverrideGravity, GravityAccel, FVector(1.f, 1.f, 1.f));

	for (int32 i = 0; i < Meshes.Num(); i++)
	{
		if (Meshes[i] == nullptr) { continue; }
		if (i >= Locations.Num() - 1) {
			Meshes[i]->SetVisibility(false);
			continue;
		}
		Meshes[i]->SetVisibility(true);

		FVector DeltaP = Locations[i + 1] - Locations[i];
		ScaleVisualEffect(Meshes[i], Locations[i], DeltaP, CameraLocation, Thickness);
	}
}

bool URealisticProjectileComponent::GetExitHit(FHitResult &OutHit, FVector NewLocation, FRotator Rotation, const UWorld* World) {
	if (!IsValid(UpdatedPrimitive)) { return false; }

	FComponentQueryParams CQP = FComponentQueryParams();
	CQP.AddIgnoredComponent(UpdatedPrimitive);
	CQP.bFindInitialOverlaps = true;
	CQP.bTraceComplex = false;
	CQP.bReturnPhysicalMaterial = true;

	FHitResult FirstHit = FHitResult(); // Of all the hits, the closest to start position
	FirstHit.Time = -1.f; // Anything over this is the new closest hit (we are sweeping backwards so 0=far, 1=close)
	
	for (FHitResult Current : ObjectsPenetrated)
	{
		FHitResult ThisHit;
		ThisHit.FaceIndex = 0; // TODO remove using this for debugging inf loops
		bool HasExitHit = GetExitHit(ThisHit, Current.Location, NewLocation, Rotation, World, Current.GetComponent(), CQP);

		if (HasExitHit)
		{
			if (ThisHit.Time > FirstHit.Time)
			{
				FirstHit = ThisHit;
			}
		}
	}
	OutHit = FirstHit;
	return FirstHit.Time >= 0.f;
}

void URealisticProjectileComponent::ScaleTracerMesh(UStaticMeshComponent* Mesh, const URealisticProjectileComponent* ProjectileComponent, const FVector& WorldLocation, 
		const FVector& CameraLocation, float Thickness, float LengthFactor)
{
	if (ProjectileComponent == nullptr) { return; }
	float VSize = ProjectileComponent->Velocity.Size();

	float DistanceToLastHit = (WorldLocation - ProjectileComponent->InitialConditions.InitialWorldLocation).Size();
	float DesiredLength = VSize * LengthFactor;
	float FinalLength = fminf(0.75f * DistanceToLastHit, DesiredLength);
	float RelativeSize = FinalLength / VSize;
	FVector TracerVec = -RelativeSize * ProjectileComponent->Velocity;
	ScaleVisualEffect(Mesh, WorldLocation, TracerVec, CameraLocation, Thickness);
}

void URealisticProjectileComponent::ScaleVisualEffect(UStaticMeshComponent* Mesh, const FVector& WorldLocation, const FVector& Direction, const FVector& CameraLocation, float Thickness)
{
	if (!Mesh) { return; }
	float DistanceFromCamera = FMath::PointDistToSegment(CameraLocation, WorldLocation, WorldLocation + Direction);
	float YZScale = Thickness * FMath::GetMappedRangeValueClamped(FVector2D(0.f, 1000000.f), FVector2D(1.f, 300.f), DistanceFromCamera);
	if (YZScale == 0.f) { YZScale = 0.001f; }
	// Small or big scales cause errors
	float XScale = FMath::Clamp<float>(Direction.Size(), 0.1f, 5000000.f);
	FVector FinalScale = FVector(XScale, YZScale, YZScale);
	FRotator Rotation = Direction.ToOrientationRotator();
	Mesh->SetWorldTransform(FTransform(Rotation, WorldLocation, FinalScale));
}

void URealisticProjectileComponent::SetInitialConditions(FTrajectoryInitialConditions Trajectory)
{
	InitialConditionHistory.Emplace(Trajectory);
	InitialConditions = Trajectory;
}

void URealisticProjectileComponent::SetVelocityInLocalSpace(FVector NewVelocity)
{
	if (IsValid(UpdatedComponent))
	{
		Velocity = UpdatedComponent->GetComponentToWorld().TransformVectorNoScale(NewVelocity);
	}
}

bool URealisticProjectileComponent::ShouldBounce(const FHitResult& Hit, const FVector& ImpactVelocity)
{
	// Get Toughness
	float Toughness = DEFAULT_TOUGHNESS;
	if (IsValid(MaterialPropertiesTable))
	{
		UPhysicalMaterial* PhysMat = Hit.PhysMaterial.Get();
		if (PhysMat)
		{
			static const FString ContextString(TEXT("GENERAL"));
			FMaterialProperties* MP = MaterialPropertiesTable->FindRow<FMaterialProperties>(FName(*PhysMat->GetName()), ContextString);
			if (MP)
			{
				Toughness = MP->Toughness;
			}
		}
	}

	return InternalShouldBounce(this, Hit, Toughness, ImpactVelocity);
}

void URealisticProjectileComponent::StopSimulating(const FHitResult& HitResult, bool PredictOnly)
{
	SetUpdatedComponent(NULL);
	if (!PredictOnly) { OnProjectileStop.Broadcast(HitResult); }
}

bool URealisticProjectileComponent::ServerCheckClientHit(UObject* WorldContextObject, FVector& OutImpactVelocity, TArray<AActor*> ActiveProjectiles, FHitResult Hit, float TimeSinceShot, 
	int32 BulletRandSeed, float Epsilon, float StepTime, bool DrawHit, bool bOverrideGravity, FVector NewGravity)
{
	double starttime = FPlatformTime::Seconds();
	if (TimeSinceShot < 0.f) { return false; }
	
	UWorld* World = nullptr;
	if (WorldContextObject == nullptr || (World = WorldContextObject->GetWorld()) == nullptr) { return false; }
	float GameTime = World->GetTimeSeconds();

	// Take this opportunity to clear out the dead pointers
	ActiveProjectiles.RemoveAll([](AActor* Ptr) { return Ptr == nullptr || Ptr->IsActorBeingDestroyed(); });

	// Find the bullet
	AActor* FoundBullet = FindProjectileByRandSeedAndTimeShot(World, ActiveProjectiles, BulletRandSeed, TimeSinceShot);
	if (FoundBullet == nullptr) { return false; }
	URealisticProjectileComponent* RPComponent = Cast<URealisticProjectileComponent>(FoundBullet->GetComponentByClass(URealisticProjectileComponent::StaticClass()));
	if (RPComponent == nullptr) { return false; }

	FTrajectoryInitialConditions r0 = RPComponent->InitialConditionHistory[0];

	// Recalculate the trajectory and see where we have it at time-
	TArray<FVector> WorldLocations;
	FVector FinalVelocity;
	FRealisticProjectileBehavior Settings = RPComponent->Settings;
	FTrajectoryInitialConditions Start = RPComponent->InitialConditionHistory.Num() ? RPComponent->InitialConditionHistory[0] : FTrajectoryInitialConditions();
	// Get a step time around the one given, that will give us exactly the moment we want.
	int32 NumSteps = (int32)(TimeSinceShot / StepTime);
	if (NumSteps < 4) { NumSteps = 4; }
	float AdjustedStep = (TimeSinceShot - SMALL_NUMBER) / NumSteps;
	GetPredictedTrajectory(WorldContextObject, WorldLocations, FinalVelocity, Start.InitialWorldLocation, Start.InitialRotation, FoundBullet->GetClass(),
		BulletRandSeed, TimeSinceShot, AdjustedStep, 64, bOverrideGravity, NewGravity, FVector(1, 1, 1));

#if WITH_EDITOR
	if (DrawHit && WorldLocations.Num() > 0)
	{
		DrawDebugSphere(World, WorldLocations.Last(), 5.f, 32, FColor::Green, false, 20.f);
		DrawDebugSphere(World, Hit.Location, 10.f, 32, FColor::White, false, 20.f);
	}
#endif

	OutImpactVelocity = FinalVelocity;
	return WorldLocations.Num() > 0 && (WorldLocations.Last() - Hit.Location).SizeSquared() < Epsilon * Epsilon;
}

FVector URealisticProjectileComponent::PositionAtTime_LinearDrag(const FTrajectoryInitialConditions& TIC, float k, const FVector& Vinf, float TimeSinceLaunch)
{
	FVector numerator = (TIC.InitialVelocity + k * TimeSinceLaunch * Vinf) * TimeSinceLaunch;
	float denominator = 1 + k * TimeSinceLaunch;
	return (numerator / denominator) + TIC.InitialWorldLocation;
}

float URealisticProjectileComponent::TimeOfFlight_LinearDrag(const FTrajectoryInitialConditions& TIC, float k, const FVector& Vinf, const FVector& EndLocation, const FVector& CurrentVelocity)
{
	FVector DeltaLoc = EndLocation - TIC.InitialWorldLocation;

	// We only need to evaluate one axis, so decide which one will give accurate results.
	FVector QuantityToDecideAxisWith = DeltaLoc.SizeSquared() < SMALL_NUMBER ? TIC.InitialVelocity : DeltaLoc;
	QuantityToDecideAxisWith = QuantityToDecideAxisWith.SizeSquared() < SMALL_NUMBER ? Vinf : QuantityToDecideAxisWith;
	FVector AxisToUse = URealisticProjectileComponent::GetLargestDimension(QuantityToDecideAxisWith);

	// Signed magnitude of the values we are interested in - delta location, initial velocity, TerminalV
	float dp = 0.f, v = 0.f, TermV = 0.f;
	for (uint8 i = 0; i < 3; i++)
	{
		dp += DeltaLoc[i] * AxisToUse[i];
		v += TIC.InitialVelocity[i] * AxisToUse[i];
		TermV += Vinf[i] * AxisToUse[i];
	}

	if (fabsf(TermV) < SMALL_NUMBER || fabsf(k) < SMALL_NUMBER)
	{
		// 0 = k.Vinf.t^2 + (v0 - dp.k)t - dp
		// t = dp / (v0 - dp.k)
		//UE_LOG(ProjectilePhysics, Display, TEXT("Got air flight time: %f. dp=%f v=%f Vinf=%f"), dp / (v - dp * k), dp, v, TermV)
		if (fabsf(v - dp * k) < SMALL_NUMBER) { return 0.f; }
		return dp / (v - dp * k);
	}
	else
	{
		//Hard mode - there are two possible answers
		// dp = (v0t + k Vinf t^2) / (1 + kt)		:Multiply by 1 + kt
		// dp + dp.k.t = v0t + k Vinf t^2			:Put all terms on right
		// 0 = k.Vinf.t^2 + (v0 - dp.k)t - dp
		float a2 = 2.f * k * TermV; // 2 * 'a' term
		float b = v - dp * k;
		float c = -dp;
		//solve quadratic formula
		float t1 = (-b + sqrtf(b*b - 2.f * a2 * c)) / a2;
		float t2 = (-b - sqrtf(b*b - 2.f * a2 * c)) / a2;
		//UE_LOG(ProjectilePhysics, Display, TEXT("Got air flight time: %f %f. dp=%f v=%f Vinf=%f"), t1, t2, dp, v, TermV)
		// Maybe one answer is in -ve time
		if (t1 < 0.f) return t2;
		if (t2 < 0.f) return t1;
		// So they are both in +ve time. We can use current velocity to tell if it is the one before the apex or after.
		float CurrentDirection = 0.f;
		for (uint8 i = 0; i < 3; i++) { CurrentDirection += CurrentVelocity[i] * AxisToUse[i]; }
		if (CurrentDirection * TermV < 0.f)
		{
			// Before apex, get earlier time
			return t1 < t2 ? t1 : t2;
		}
		else
		{
			// After apex, get later time
			return t1 > t2 ? t1 : t2;
		}
	}
}

float URealisticProjectileComponent::TimeOfFlight_Sliding(const FTrajectoryInitialConditions& TIC, const FVector& EndLocation, const FVector& CurrentVelocity)
{
	FVector DeltaLoc = EndLocation - TIC.InitialWorldLocation;

	// We only need to evaluate one axis, so decide which one will give accurate results.
	FVector QuantityToDecideAxisWith = DeltaLoc.SizeSquared() < SMALL_NUMBER ? TIC.InitialVelocity : DeltaLoc;
	QuantityToDecideAxisWith = QuantityToDecideAxisWith.SizeSquared() < SMALL_NUMBER ? TIC.SlidingAcceleration : QuantityToDecideAxisWith;
	FVector AxisToUse = URealisticProjectileComponent::GetLargestDimension(QuantityToDecideAxisWith);

	// Signed magnitude of the values we are interested in - delta location, initial velocity, acceleration
	float dp = 0.f, v = 0.f, Acceleration = 0.f;
	for (uint8 i = 0; i < 3; i++)
	{
		dp += DeltaLoc[i] * AxisToUse[i];
		v += TIC.InitialVelocity[i] * AxisToUse[i];
		Acceleration += TIC.SlidingAcceleration[i] * AxisToUse[i];
	}
	// Easy case, no accel
	if (fabsf(Acceleration) < SMALL_NUMBER)
	{
		//UE_LOG(ProjectilePhysics, Display, TEXT("Got sliding time: %f. dp=%f dp2=%f v=%f v2=%f,%f,%f Axis=%f,%f,%f, Accel=%f,%f,%f "), dp / v, dp, (DeltaLoc * AxisToUse).Size(), v, TIC.InitialVelocity.X, TIC.InitialVelocity.Y, TIC.InitialVelocity.Z,
		//	 AxisToUse.X, AxisToUse.Y, AxisToUse.Z, TIC.SlidingAcceleration.X, TIC.SlidingAcceleration.Y, TIC.SlidingAcceleration.Z)
		if (fabsf(v) < SMALL_NUMBER && fabsf(dp) < SMALL_NUMBER) { return 0.f; } // No accel, no velocity
		return dp / v;
	}

	// 0 = 0.5at^2 + v0t -dp
	float a = 0.5f * Acceleration, b = v, c = -dp;
	float t1 = (-b + sqrtf(b * b - 4.f * a * c)) / (2.f * a);
	float t2 = (-b - sqrtf(b * b - 4.f * a * c)) / (2.f * a);
	//UE_LOG(ProjectilePhysics, Display, TEXT("Got sliding time: %f %f. dp=%f v=%f a=%f"), t1, t2, dp, v, a)
	// Maybe one answer is in -ve time
	if (t1 < 0.f) return t2;
	if (t2 < 0.f) return t1;
	// So they are both in +ve time. We can use current velocity to tell if it is the one before the apex or after.
	float CurrentDirection = 0.f;
	for (uint8 i = 0; i < 3; i++) { CurrentDirection += CurrentVelocity[i] * AxisToUse[i]; }
	if (CurrentDirection * Acceleration < 0.f) 
	{
		// Before apex, get earlier time
		return t1 < t2 ? t1 : t2;
	}
	else
	{
		// After apex, get later time
		return t1 > t2 ? t1 : t2;
	}
}

FVector URealisticProjectileComponent::VelocityAtTime_LinearDrag(const FTrajectoryInitialConditions& TIC, float k, const FVector& Vinf, float TimeSinceLaunch)
{
	float denominator = (1 + k * TimeSinceLaunch);
	denominator *= denominator;
	FVector numerator = TIC.InitialVelocity + k * TimeSinceLaunch * (2.f + k * TimeSinceLaunch) * Vinf;
	FVector NewVelocity = numerator / denominator;
	return NewVelocity;
}

void URealisticProjectileComponent::GetPredictedTrajectory(UObject* WorldContextObject, TArray<FVector>& WorldLocations, FVector& LastVelocity, FVector StartLocation, FRotator StartRotation, 
	TSubclassOf<AActor> Properties, int32 RandSeed, float TimeToSimulate, float StepTime, int32 MaxPoints, bool bOverrideGravity, FVector GravityAccel, FVector Scale)
{
	InternalGetPredictedTrajectoryForSettings(WorldContextObject, WorldLocations, LastVelocity, StartLocation, StartRotation, Properties, nullptr, RandSeed, TimeToSimulate, StepTime, MaxPoints,
		bOverrideGravity, GravityAccel, Scale);
}

void URealisticProjectileComponent::GetPredictedTrajectoryFull(UObject* WorldContextObject, TArray<FVector>& WorldLocations, TArray<FRotator>& Rotations, TArray<FVector>& Velocities, 
	FVector StartLocation, FRotator StartRotation, TSubclassOf<AActor> Properties, int32 RandSeed, float TimeToSimulate, float StepTime, int32 MaxPoints,
	bool bOverrideGravity, FVector GravityAccel, FVector Scale)
{
	
	AActor* Instance;
	URealisticProjectileComponent* SpawnedComponent;
	UWorld* World;
	if (!SetupPredictedTrajectory(WorldContextObject, Instance, SpawnedComponent, World, StartLocation, StartRotation, Properties, RandSeed, bOverrideGravity, GravityAccel, Scale) ||
		TimeToSimulate <= 0.f || MaxPoints < 1 || StepTime <= 0.f) 
	{
		return;
	}

	FVector Dummy;
	InternalPredictTrajectory(World, WorldLocations, Rotations, Velocities, Dummy, Instance, SpawnedComponent, StartLocation, StartRotation, TimeToSimulate, StepTime, MaxPoints, false);
}

void URealisticProjectileComponent::GetPredictedTrajectoryForSettings(UObject* WorldContextObject, TArray<FVector>& WorldLocations, FVector StartLocation, 
	FRotator StartRotation, TSubclassOf<AActor> Properties, FRealisticProjectileBehavior NewSettings, int32 RandSeed, float TimeToSimulate, float StepTime, 
	int32 MaxPoints, bool bOverrideGravity, FVector GravityAccel, FVector Scale)
{
	FVector Dummy;
	InternalGetPredictedTrajectoryForSettings(WorldContextObject, WorldLocations, Dummy, StartLocation, StartRotation, Properties, &NewSettings,
		RandSeed, TimeToSimulate, StepTime, MaxPoints, bOverrideGravity, GravityAccel, Scale);
}

/********************************************************
 ******		Protected Functions				************/
void URealisticProjectileComponent::ActorMove(const FVector& NewLocation, const FRotator& NewRotation)
{
	if (IsValid(UpdatedComponent))
	{
		UpdatedComponent->SetWorldLocationAndRotation(
			NewLocation,
			NewRotation,
			false);
	}
}

FVector URealisticProjectileComponent::AdjustDirection(FVector InVelocity, FVector ImpactNormal)
{
	float max_anglechange = Settings.ExitRandomness * FMath::GetMappedRangeValueClamped(FVector2D(5000.f, 99900.f), FVector2D(0.f, 10.f), InVelocity.Size());
	float pitch = (RandStream.FRand() - 0.5f) * max_anglechange;
	float yaw = (RandStream.FRand() - 0.5f) * max_anglechange;
	//float roll = (RandStream.FRand() - 0.5f) * max_anglechange;
	FRotator rot = FRotator(pitch, yaw, 0.f);
	FVector PossibleAnswer = rot.RotateVector(InVelocity);
	float dotp = FVector::DotProduct(PossibleAnswer, -ImpactNormal);
	if (dotp >= 0.f)
	{
		PossibleAnswer = PossibleAnswer.MirrorByVector(ImpactNormal);
	}
	return PossibleAnswer;
}

/** Apply impulse to other object after hit*/
void URealisticProjectileComponent::ApplyImpulse(FHitResult& Hit, FVector ImpactVelocity) const
{
	UPrimitiveComponent* HitComponent = Hit.GetComponent();
	if (IsValid(HitComponent) && fabsf(ImpulseScale) > SMALL_NUMBER)
	{
		if (IsValid(UpdatedPrimitive) && fabsf(ImpulseScale) > SMALL_NUMBER && HitComponent->IsSimulatingPhysics())
		{
			//Impart momentum
			float Mass = UpdatedPrimitive->CalculateMass();
			FVector Impulse = Mass * ImpactVelocity * ImpulseScale;
			HitComponent->AddImpulseAtLocation(Impulse, Hit.ImpactPoint);
		}
	}
}

void URealisticProjectileComponent::BroadcastHitOrOverlap(FHitResult Hit)
{
	AActor* MyActor = this->GetOwner();
	AActor* OtherActor = Hit.GetActor();
	UPrimitiveComponent* OtherComponent = Hit.GetComponent();
	if (IsValid(UpdatedPrimitive) && IsValid(OtherComponent) && IsValid(OtherActor) && IsValid(MyActor))
	{
		float TimeSinceLaunch = ComputeTimeOfFlight(InitialConditions, Hit.Location, Velocity);
		ECollisionResponse CR = UpdatedPrimitive->GetCollisionResponseToComponent(OtherComponent);
#if ENGINE_MINOR_VERSION <= 19
		bool OverlapsIsOn = UpdatedPrimitive->bGenerateOverlapEvents && OtherComponent->bGenerateOverlapEvents;
#else
		bool OverlapsIsOn = UpdatedPrimitive->GetGenerateOverlapEvents() && OtherComponent->GetGenerateOverlapEvents();
#endif
		if (CR < ECollisionResponse::ECR_Block && OverlapsIsOn) {
			OtherComponent->OnComponentBeginOverlap.Broadcast(OtherComponent, MyActor, UpdatedPrimitive, UpdatedPrimitive->GetBodyInstance()->InstanceBodyIndex, true, FHitResult::GetReversedHit(Hit));
			OtherActor->OnActorBeginOverlap.Broadcast(OtherActor, MyActor); // The OnActor... delegates don't seem to work and I don't know why
			
			FBodyInstance* OtherBody = OtherComponent->GetBodyInstance();
			int32 BodyIndex = OtherBody == nullptr ? 0 : OtherBody->InstanceBodyIndex;
			UpdatedPrimitive->OnComponentBeginOverlap.Broadcast(UpdatedPrimitive, OtherActor, OtherComponent, BodyIndex, true, Hit);
			MyActor->OnActorBeginOverlap.Broadcast(MyActor, OtherActor); // The OnActor... delegates don't seem to work and I don't know why
			float TimeSinceSpawn = (TimeSinceLaunch + InitialConditions.t0) - InitialConditionHistory[0].t0;
			OnComponentBeginOverlap.Broadcast(Hit, InitialConditionHistory.Num(), TimeSinceSpawn);
		}
		else if (CR >= ECollisionResponse::ECR_Block)
		{
			OnProjectileAnyHit.Broadcast(Hit, ComputeNewVelocity(TimeSinceLaunch));
			OtherComponent->OnComponentHit.Broadcast(OtherComponent, MyActor, UpdatedPrimitive, FVector::ZeroVector, FHitResult::GetReversedHit(Hit));
			OtherActor->OnActorHit.Broadcast(OtherActor, MyActor, FVector(0,0,0), Hit); // The OnActor... delegates don't seem to work and I don't know why
			UpdatedPrimitive->OnComponentHit.Broadcast(UpdatedPrimitive, OtherActor, OtherComponent, FVector::ZeroVector, Hit);
			MyActor->OnActorHit.Broadcast(MyActor, OtherActor, FVector::ZeroVector, Hit); // The OnActor... delegates don't seem to work and I don't know why
		}
	}
}

float URealisticProjectileComponent::ComputeTimeOfFlight(const FTrajectoryInitialConditions& TInitialConditions, const FVector& HitLocation, const FVector& CurrentVelocity)
{
	switch (TInitialConditions.TrajectoryType)
	{
		case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Air:
			return fmaxf(TimeOfFlight_LinearDrag(TInitialConditions, k, Vinf, HitLocation, CurrentVelocity), 0.f);
		case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Object:
		{
			float p = (HitLocation - TInitialConditions.InitialWorldLocation).Size();
			float accel = -TInitialConditions.PenDeceleration;
			// solve 0 = 0.5at^2 + v0t - p   using quadratic formula for +ve t
			float a = 0.5f * accel;
			float b = TInitialConditions.v0length;
			float c = -p;
			float Numerator = -b + sqrtf(b * b - 4.f * a * c);
			float Result = Numerator / (2.f * a);
			return fmaxf(Result, 0.f);
		}
		case FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding:
			return fmaxf(TimeOfFlight_Sliding(TInitialConditions, HitLocation, CurrentVelocity), 0.f);
		default:
			return 0.f;
	}
}

bool URealisticProjectileComponent::GetExitHit(FHitResult &OutHit, const FVector &OrigLocation, const FVector &NewLocation, const FRotator &Rotation,
	const UWorld* World, const UPrimitiveComponent* Component, FComponentQueryParams &IgnoredComponents, uint8 RecursionDepth)
{
	if (!IsValid(UpdatedPrimitive) || !IsValid(Component) || RecursionDepth > 64)
	{
		return false;
	}

	IgnoredComponents.bFindInitialOverlaps = true;

	// Sweep until the hit results contain this object, and it is the shortest sweep that gets a hit
	for (uint8 i = 0; i < 100; i++) // Unlikely there will be 100 objects between NewLoc and OrigLoc
	{
		TempHitResults.Reset();
		World->ComponentSweepMulti(
			TempHitResults,			//results
			UpdatedPrimitive,	//Component to sweep
			NewLocation,			//start location
			OrigLocation,			//end location
			Rotation,
			IgnoredComponents		//Parameters
		);

		// Remove initial overlaps
		TempHitResults.RemoveAll([Component](const FHitResult& Hit) {return Hit.bStartPenetrating; });

		if (TempHitResults.Num() == 0)
		{
			// Done searching - no exit hits
			IgnoredComponents.ClearIgnoredComponents();
			return false;
		}

		// Test to see if the sweep got any hits with the same component as the queried one.
		// Be careful with TempHitResults. Other functions will modify it.
		FHitResult* PossibleClosestHit = TempHitResults.FindByPredicate([Component](const FHitResult& Hit) {return Hit.GetComponent() == Component; });
		if (PossibleClosestHit != nullptr)
		{
			FHitResult FoundHit = *PossibleClosestHit;
			// We found a hit. Recursively see if there's a closer hit.
			FHitResult CloserHit;
			FVector CloserHitStart = FoundHit.Location + (OrigLocation - FoundHit.Location).GetSafeNormal() * NUDGE_DISTANCE;
			bool bIsCloserHit = GetExitHit(CloserHit, OrigLocation, CloserHitStart, Rotation, World, Component, IgnoredComponents, RecursionDepth + 1);
			OutHit = bIsCloserHit ? CloserHit : FoundHit;
			IgnoredComponents.ClearIgnoredComponents();
			return true;
		}
		else {
			// We probably got blocked by something in the way - ignore it (blocking hits stop traces after that point)
			for (FHitResult Hit : TempHitResults)
			{
				IgnoredComponents.AddIgnoredComponent(Hit.GetComponent());
			}
		}
	}
	return false; // never reached
}

float URealisticProjectileComponent::GetNewPenetrationDeceleration(FHitResult Hit)
{
	if (ObjectsPenetrated.Num() == 0) return 1.f;
	UPhysicalMaterial* PM = Hit.PhysMaterial.Get();
	FString hitmaterial = PM->GetName();
	/*if(GEngine){
	GEngine->AddOnScreenDebugMessage(-1,
	15.f,
	FColor::Green,
	hitmaterial);
	}*/

	if (IsValid(MaterialPropertiesTable))
	{
		static const FString ContextString(TEXT("GENERAL"));

		FMaterialProperties* LookupRow = MaterialPropertiesTable->FindRow<FMaterialProperties>(*hitmaterial, ContextString);

		if (LookupRow)
		{
			return InternalGetPenetrationDeceleration(LookupRow->Toughness, Settings.DecelerationExponent, Settings.DecelerationCoefficient);
		}
	}
	return InternalGetPenetrationDeceleration(DEFAULT_TOUGHNESS, Settings.DecelerationExponent, Settings.DecelerationCoefficient);
}

void URealisticProjectileComponent::NotifyHitObjectOfEndOverlap(UPrimitiveComponent* OtherComponent)
{
	AActor* MyActor = this->GetOwner();
	AActor* OtherActor;
#if ENGINE_MINOR_VERSION <= 19
	bool OverlapsIsOn = UpdatedPrimitive->bGenerateOverlapEvents && OtherComponent->bGenerateOverlapEvents;
#else
	bool OverlapsIsOn = UpdatedPrimitive->GetGenerateOverlapEvents() && OtherComponent->GetGenerateOverlapEvents();
#endif
	if (IsValid(OtherComponent) && IsValid(UpdatedPrimitive) && IsValid(MyActor) && IsValid(OtherActor = OtherComponent->GetOwner()) &&
			OverlapsIsOn)
	{
		OtherComponent->OnComponentEndOverlap.Broadcast(OtherComponent, MyActor, UpdatedPrimitive, UpdatedPrimitive->GetBodyInstance()->InstanceBodyIndex);
		OtherActor->OnActorEndOverlap.Broadcast(OtherActor, MyActor); // The OnActor... delegates don't seem to work and I don't know why

		FBodyInstance* OtherBody = OtherComponent->GetBodyInstance();
		int32 BodyIndex = OtherBody == nullptr ? 0 : OtherBody->InstanceBodyIndex;
		UpdatedPrimitive->OnComponentEndOverlap.Broadcast(UpdatedPrimitive, OtherActor, OtherComponent, BodyIndex);
		MyActor->OnActorEndOverlap.Broadcast(MyActor, OtherActor); // The OnActor... delegates don't seem to work and I don't know why
	}
}

/********************************************************
 ******		Private Functions				************/
bool URealisticProjectileComponent::CheckStillInWorld()
{
	if (!UpdatedComponent)
	{
		return false;
	}

	const UWorld* MyWorld = GetWorld();
	if (!MyWorld)
	{
		return false;
	}

	// check the variations of KillZ
	AWorldSettings* WorldSettings = MyWorld->GetWorldSettings(true);
	if (!WorldSettings->bEnableWorldBoundsChecks)
	{
		return true;
	}
	AActor* ActorOwner = UpdatedComponent->GetOwner();
	if (!IsValid(ActorOwner))
	{
		return false;
	}
	if (ActorOwner->GetActorLocation().Z < WorldSettings->KillZ)
	{
		UDamageType const* DmgType = WorldSettings->KillZDamageType ? WorldSettings->KillZDamageType->GetDefaultObject<UDamageType>() : GetDefault<UDamageType>();
		ActorOwner->FellOutOfWorld(*DmgType);
		return false;
	}
	// Check if box has poked outside the world
	else if (UpdatedComponent && UpdatedComponent->IsRegistered())
	{
		const FBox&	Box = UpdatedComponent->Bounds.GetBox();
		if (Box.Min.X < -HALF_WORLD_MAX || Box.Max.X > HALF_WORLD_MAX ||
			Box.Min.Y < -HALF_WORLD_MAX || Box.Max.Y > HALF_WORLD_MAX ||
			Box.Min.Z < -HALF_WORLD_MAX || Box.Max.Z > HALF_WORLD_MAX)
		{
			//UE_LOG(LogProjectileMovement, Warning, TEXT("%s is outside the world bounds!"), *ActorOwner->GetName());
			ActorOwner->OutsideWorldBounds();
			// not safe to use physics or collision at this point
			ActorOwner->SetActorEnableCollision(false);
			FHitResult Hit(1.f);
			StopSimulating(Hit);
			return false;
		}
	}
	return true;
}

bool URealisticProjectileComponent::CheckStillSliding(struct SubTickState& OutState, const struct SubTickState& PotentialState, const UWorld* World, float GameTime, float TimeSinceLaunch)
{
	// Sweep component into sliding surface to check if it's still there
	UPrimitiveComponent* Surface = InitialConditions.GetSlidingSurface(), *Surface2 = InitialConditions.GetSlidingSurface2();
	const float CheckDepth = Surface2 == nullptr ? SLIDING_CHECK_DEPTH : 1.5f * SLIDING_CHECK_DEPTH;
	FVector CheckLocation = PotentialState.Location - InitialConditions.SlidingSurfaceNormal * CheckDepth;
	FHitResult Tmp; // Not used, just needed for GetExitHit()
	FComponentQueryParams CQP = FComponentQueryParams();
	CQP.bFindInitialOverlaps = false;

	if (Surface == nullptr) { return false; }
	bool Surface1StillThere = GetExitHit(Tmp, CheckLocation, PotentialState.Location, PotentialState.Rotation, World, Surface, CQP, 0);
	bool Surface2There = Surface2 != nullptr && GetExitHit(Tmp, CheckLocation, PotentialState.Location, PotentialState.Rotation, World, Surface2, CQP, 0);

	return (Surface1StillThere && Surface2 == nullptr) || (Surface1StillThere && Surface2There);	
}

#if WITH_EDITOR
void URealisticProjectileComponent::DoDrawDebugLine(FVector StartLocation, FVector NewLocation, float inVelocity)
{
	if (bDrawDebugLine)
	{
		const UWorld* MyWorld = GetWorld();
		if (!MyWorld)
		{
			return;
		}
		FColor TraceColor = DebugLineAuxiliaryColor;
		if (bDebugLineColorFromVelocity)
		{
			//Calculate color from velocity
			float fraction = inVelocity / Settings.InitialSpeed;
			int32 i_col = (int32)(fraction * 255 * 4);
			uint8 r = 255, g = 0, b = 0;

			if (i_col >= 765 && i_col <= 1020)
			{
				g = 1020 - i_col;
			}
			else if (i_col >= 510 && i_col <= 765)
			{
				g = 255;
				r = 765 - i_col;
			}
			else if (i_col >= 255 && i_col <= 510)
			{
				g = i_col - 510;
				r = 0;
				b = 510 - i_col;
			}
			else if (i_col >= 0)
			{
				b = 255;
				r = 255 - i_col;
				g = r;
			}
			TraceColor = FColor(r, g, b);
		}

		DrawDebugLine(MyWorld,
			StartLocation,
			NewLocation,
			TraceColor,
			true,
			DebugLineDuration,
			(uint8)'\000',
			DebugLineThickness);
	}
}
#endif // WITH_EDITOR

/** Sweep the updated component geometry, returning all hitresults in the parameter
* Does not move the updated component */
bool URealisticProjectileComponent::DoSweep(FVector StartLocation,
	FVector EndLocation,
	FRotator Rot,
	TArray<struct FHitResult> &OutHits,
	const UWorld* World,
	bool KeepOverlaps,
	bool GetPhysicalMaterial)
{
	if (!IsValid(UpdatedPrimitive)) return false;

	FComponentQueryParams CQP = FComponentQueryParams();
	CQP.AddIgnoredComponent(UpdatedPrimitive);
	CQP.bFindInitialOverlaps = true;
	CQP.bReturnPhysicalMaterial = GetPhysicalMaterial;
	CQP.bTraceComplex = false;

	World->ComponentSweepMulti(
		OutHits,			//results
		UpdatedPrimitive,	//Component to sweep
		StartLocation,			//start location
		EndLocation,			//end location
		Rot,
		CQP					//Parameters
		);

	if (!KeepOverlaps)
	{
		OutHits.RemoveAll([this](const FHitResult& Hit)
		{
			UPrimitiveComponent* OtherComponent = Hit.GetComponent();
			return OtherComponent == nullptr || UpdatedPrimitive->GetCollisionResponseToComponent(OtherComponent) < ECollisionResponse::ECR_Block;
		});
	}
	return OutHits.Num() > 0;
}

AActor* URealisticProjectileComponent::FindProjectileByRandSeedAndTimeShot(const UWorld* World, const TArray<AActor*>& ProjectileActors, int32 RandSeed, float TimeSinceShot)
{
	TArray<AActor*> Matches = ProjectileActors.FilterByPredicate([RandSeed](const AActor* Instance)
	{
		if ( Instance != nullptr)
		{
			URealisticProjectileComponent* RPC = Cast<URealisticProjectileComponent>(Instance->GetComponentByClass(URealisticProjectileComponent::StaticClass()));
			return RPC && RPC->RandStream.GetInitialSeed() == RandSeed;
		}
		return false;
	});

	if (Matches.Num() == 0) { return nullptr; }
	else if (Matches.Num() > 1) // 2 active bullets have the same random seed, wow!
	{
		float ServerApproxTimeShot = World->GetTimeSeconds() - TimeSinceShot;
		Algo::Sort(Matches, [ServerApproxTimeShot](const AActor* A, const AActor* B)
		{
			URealisticProjectileComponent* ARPC = Cast<URealisticProjectileComponent>(A->GetComponentByClass(URealisticProjectileComponent::StaticClass()));
			URealisticProjectileComponent* BRPC = Cast<URealisticProjectileComponent>(B->GetComponentByClass(URealisticProjectileComponent::StaticClass()));
			if (ARPC && BRPC && ARPC->InitialConditionHistory.Num() > 0 && BRPC->InitialConditionHistory.Num() > 0)
			{
				// Find time diff for A and time diff for B and return true if A should precede B
				float AMinusB = fabsf(ARPC->InitialConditionHistory[0].t0 - ServerApproxTimeShot) - fabsf(BRPC->InitialConditionHistory[0].t0 - ServerApproxTimeShot);
				return (AMinusB > 0);
			}
			else { return true; }
		});
	}

	return Matches[0];
}

FVector URealisticProjectileComponent::GetLargestDimension(const FVector& Vector)
{
	FVector AbsVector = Vector * Vector.GetSignVector();
	if (AbsVector.X > AbsVector.Y && AbsVector.X > AbsVector.Z) { return FVector(1.f, 0.f, 0.f); }
	else if (AbsVector.Y > AbsVector.Z && AbsVector.Y > AbsVector.X) { return FVector(0.f, 1.f, 0.f);}
	else { return FVector(0.f, 0.f, 1.f); }
}

float URealisticProjectileComponent::InternalGetPenetrationDeceleration(float Toughness, float DecelerationExponent, float DecelerationCoefficient)
{
	return -(DecelerationCoefficient * powf(Toughness, DecelerationExponent));
}

void URealisticProjectileComponent::InternalGetPredictedTrajectoryForSettings(UObject* WorldContextObject, TArray<FVector>& WorldLocations, FVector& LastVelocity, FVector StartLocation, FRotator StartRotation, 
	TSubclassOf<AActor> Properties, const FRealisticProjectileBehavior* Settings, int32 RandSeed, float TimeToSimulate, float StepTime, int32 MaxPoints,
	bool bOverrideGravity, FVector GravityAccel, FVector Scale)
{
	AActor* Instance;
	URealisticProjectileComponent* SpawnedComponent;
	UWorld* World;
	if (TimeToSimulate <= 0.f || MaxPoints < 1 || StepTime <= 0.f ||
		!SetupPredictedTrajectory(WorldContextObject, Instance, SpawnedComponent, World, StartLocation, StartRotation, Properties, RandSeed,
			bOverrideGravity, GravityAccel, Scale, Settings)) 
	{
		return;
	}

	TArray<FVector> Dummy1;
	TArray<FRotator> Dummy2;
	InternalPredictTrajectory(World, WorldLocations, Dummy2, Dummy1, LastVelocity, Instance, SpawnedComponent, StartLocation, StartRotation, TimeToSimulate, StepTime, MaxPoints);
	
	if (Instance) { Instance->Destroy(); }
}

void URealisticProjectileComponent::InternalPredictTrajectory(UWorld* World, TArray<FVector>& WorldLocations, TArray<FRotator>& Rotations, TArray<FVector>& Velocities, FVector& LastVelocity,
		AActor* Instance, URealisticProjectileComponent* SpawnedComponent, const FVector& StartLocation, const FRotator& StartRotation, float TimeToSimulate, 
		float StepTime, int32 MaxPoints, bool LocationsOnly)
{
	
	WorldLocations.Emplace(StartLocation);
	if (!LocationsOnly)
	{
		Rotations.Emplace(StartRotation);
		Velocities.Emplace(SpawnedComponent->Settings.InitialSpeed * StartRotation.Vector());
	}

	float Time = World->GetTimeSeconds(), EndTime = Time + TimeToSimulate;
	FTrajectoryInitialConditions Trajectory = FTrajectoryInitialConditions(SpawnedComponent->Settings.InitialSpeed * StartRotation.Vector(), StartLocation, Time, StartRotation);
	struct SubTickState BeforeState = SubTickState(StartLocation, StartRotation, Trajectory.InitialVelocity, Trajectory, StepTime);
	Time += StepTime;
	for (int32 Iteration = 0; Time <= EndTime && Iteration < MaxPoints && !SpawnedComponent->HasStoppedMoving(SpawnedComponent, true) && !SpawnedComponent->HasStoppedSimulation(); Iteration++)
	{
		//UE_LOG(LogTemp, Warning, TEXT("Predict traj: Time=%f StartTime=%f TotalTime=%f, Step=%f"), Time, World->GetTimeSeconds(), TimeToSimulate, StepTime)
		struct SubTickState AfterState;
		InternalSubTick(SpawnedComponent, AfterState, BeforeState, World, Time, true);
		if (AfterState.TickRemainingTime < SMALL_NUMBER)
		{
			if (Time + StepTime > EndTime && Time < EndTime) // Make last tick exactly on endtime
			{
				AfterState.TickRemainingTime = EndTime - Time;
				Time = EndTime;
			}
			else
			{
				Time += StepTime;
				AfterState.TickRemainingTime = StepTime;
			}
		}
		BeforeState = AfterState;
		SpawnedComponent->InitialConditions = AfterState.Trajectory;
		SpawnedComponent->Velocity = AfterState.Velocity;
		WorldLocations.Emplace(AfterState.Location);
		if (!LocationsOnly)
		{
			Rotations.Emplace(AfterState.Rotation);
			Velocities.Emplace(AfterState.Velocity);
		}
	}
	LastVelocity = BeforeState.Velocity;

	if (Instance) { Instance->Destroy(); }
}

bool URealisticProjectileComponent::InternalShouldBounce(const URealisticProjectileComponent* Proj, const FHitResult& Hit, float Toughness, const FVector& Velocity, float Roll)
{
	// Find angle
	float NormalsDotProduct = (Velocity.GetSafeNormal() | -Hit.ImpactNormal);
	float AngleR = acosf(NormalsDotProduct);

	float VelocityIntoSurface = (Velocity | -Hit.ImpactNormal);

	// Handle case where values are arranged wrongly
	float MinVelocityFactor = Proj->Settings.MinPenetrationVelocityFactor;
	if (MinVelocityFactor >= Proj->Settings.AlwaysPenetrateVelocityFactor) { MinVelocityFactor = Proj->Settings.AlwaysPenetrateVelocityFactor - KINDA_SMALL_NUMBER; }

	float AngleMultiplier = FMath::GetMappedRangeValueClamped(FVector2D(Proj->Settings.AlwaysPenetrateAngle, Proj->Settings.AlwaysRicochetAngle), FVector2D(1.0f, 0.f), AngleR);
	float VelMultiplier = FMath::GetMappedRangeValueClamped(FVector2D(MinVelocityFactor, Proj->Settings.AlwaysPenetrateVelocityFactor), FVector2D(0.0f, 1.f), VelocityIntoSurface / Toughness);

	// Make angle chance not linear - nth root should raise it a bit so velocity is more important too
	float ChanceOfPenetration = powf(AngleMultiplier, 0.33f) * VelMultiplier;
	if (Roll < 0.f) { Roll = Proj->RandStream.FRand(); }

	//UE_LOG(LogTemp, Warning, TEXT("Deciding on penetration for %s:Chance=%f, Angle=%f, AnglePart=%f, VelPart=%f, Toughness=%f, Vel/Toughness=%f Roll=%f MinPenVFact=%f"),
	//	*Hit.GetActor()->GetActorLabel(), ChanceOfPenetration, AngleR, AngleMultiplier, VelMultiplier, Toughness, VelocityIntoSurface / Toughness, Roll, MinVelocityFactor)

	return !(ChanceOfPenetration >= Roll);
}

void URealisticProjectileComponent::HandleAirTick(struct SubTickState& OutState, const struct SubTickState& PotentialState, const FVector& StartTickLocation, float TimeSinceLaunch, float GameTime, const UWorld* World, bool PredictOnly)
{
	if (MoveEntryHits.Num() > 0) // There is a hit to deal with
	{
		FHitResult Hit = MoveEntryHits[0];

		// Get the conditions at hit time, unless we are doing non-deterministic sliding with friction // TODO make time go backwards for friction case
		FVector NewVelocity;
		FRotator NewRotation;
		float TempRemainingTime;

		if (PotentialState.Trajectory.TrajectoryType != FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding || Settings.Friction <= 0.f)
		{
			TimeSinceLaunch = ComputeTimeOfFlight(PotentialState.Trajectory, Hit.Location, PotentialState.Velocity);
			NewVelocity = ComputeNewVelocity(TimeSinceLaunch);
			NewRotation = ComputeNewRotation(TimeSinceLaunch);
			TempRemainingTime = GameTime - (PotentialState.Trajectory.t0 + TimeSinceLaunch);
		}
		else
		{
			NewVelocity = PotentialState.Velocity;
			NewRotation = PotentialState.Rotation;
			TempRemainingTime =  0.f; // Just consider the tick done for the end of sliding with friction.
		}

		// If already sliding in a wedge and the interior angle is below 90deg, just stop here
		if (InitialConditions.TrajectoryType == FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding &&
			InitialConditions.GetSlidingSurface2() != nullptr && 
			acosf((InitialConditions.SlidingSurfaceNormal | Hit.ImpactNormal)) > 0.5f * PI)
		{
			StopSimulating(Hit, PredictOnly);
			OutState = SubTickState(Hit.Location, NewRotation, FVector::ZeroVector, PotentialState.Trajectory, 0.f);
			return;
		}

		// Check if we should change to sliding - velocity into thing was small, no sliding along the ceiling (thing normal in opposite direction to gravity)
		if (-(NewVelocity | Hit.ImpactNormal) * Settings.Bounciness < START_SLIDING_VELOCITY && (Hit.ImpactNormal | Settings.Gravity) <= 0.f && !Hit.bStartPenetrating)
		{
			struct SubTickState AtHit = SubTickState(Hit.Location, NewRotation, NewVelocity, PotentialState.Trajectory, TempRemainingTime);
			StartSliding(OutState, AtHit, GameTime, Hit, TimeSinceLaunch, PredictOnly);
			// Broadcast a hit I guess?
			if (!PredictOnly)
			{
				BroadcastHitOrOverlap(Hit);
			}
			return;
		}

#if WITH_EDITOR
		if (bDrawDebugHits && !PredictOnly)
		{
			DrawDebugSphere(World, Hit.Location, DebugHitSphereSize, 16, FColor::Emerald, false, DebugLineDuration);
		}
#endif

		// Decide whether to bounce or penetrate. If bStartPenetrating true, just penetrate. Most of the time this won't happend, but causes trouble when nudging puts
		// projectiles in other objects
		if (ShouldBounce(Hit, NewVelocity) && !Hit.bStartPenetrating)
		{
			// Nudge in reverse direction to the sweep. Usually we nudge with just the normal but this was causing trouble with infinitely bouncing slides
			float ThisNudge = PotentialState.Trajectory.TrajectoryType == FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding ? SLIDING_HOVER_HEIGHT : NUDGE_DISTANCE;
			FVector NewLocation = Hit.Location + (Hit.TraceStart - Hit.TraceEnd).GetSafeNormal() * ThisNudge;

			if (!PredictOnly)
			{
				// Broadcast event so blueprints know about them
				OnProjectileBounce.Broadcast(Hit, NewVelocity);
				BroadcastHitOrOverlap(Hit);
			}
			
			// Calculate velocity after bounce
			float VelocityIntoSurface = -(NewVelocity | Hit.ImpactNormal);
			FVector GlanceVelocity = FVector::VectorPlaneProject(NewVelocity, Hit.ImpactNormal);
			float OneMinusBounciness = 1 - Settings.Bounciness;
			NewVelocity = NewVelocity.MirrorByVector(Hit.ImpactNormal) - (OneMinusBounciness * VelocityIntoSurface * Hit.ImpactNormal);
			NewVelocity -= 0.1f * GlanceVelocity;
			NewVelocity = AdjustDirection(NewVelocity, Hit.ImpactNormal);

			FTrajectoryInitialConditions Bounced = FTrajectoryInitialConditions(NewVelocity, NewLocation,  InitialConditions.t0 + TimeSinceLaunch, NewRotation);
			OutState = SubTickState(NewLocation, NewRotation, NewVelocity, Bounced, TempRemainingTime);
		}
		else // Penetrate
		{
			// Nudge the projectile into the object so it is initially overlapping next tick
			// Failure to do this would mean a recursive loop of hitting the object
			FVector AdjustedLocation = Hit.Location - Hit.ImpactNormal * NUDGE_DISTANCE;

			// Add this hit to the list of objects we are currently 'inside of'
			ObjectsPenetrated.Emplace(Hit);
			//SetInitialConditions(Velocity, FinalLocation, InitialConditions.t0 + TimeSinceLaunch, NewRotation, GetNewPenetrationDeceleration(Hit));
			FTrajectoryInitialConditions Penetrated = FTrajectoryInitialConditions(NewVelocity, AdjustedLocation, InitialConditions.t0 + TimeSinceLaunch, NewRotation, GetNewPenetrationDeceleration(Hit));
			OutState = SubTickState(AdjustedLocation, NewRotation, NewVelocity, Penetrated, TempRemainingTime);

			if (!PredictOnly)
			{
				// Notify the relevant blueprint calls
				OnProjectilePenetrate.Broadcast(Hit, NewVelocity);
				BroadcastHitOrOverlap(Hit);
			}
		}
		if (!PredictOnly) { ApplyImpulse(Hit, NewVelocity); }
	}
	else //No hits
	{

		if (InitialConditions.TrajectoryType == FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding &&
			!CheckStillSliding(OutState, PotentialState, World, GameTime, TimeSinceLaunch))
		{
			// No longer sliding
			StopSliding(OutState, PotentialState, StartTickLocation, World, GameTime);
		}
		else
		{
			OutState = SubTickState(PotentialState.Location, PotentialState.Rotation, PotentialState.Velocity, InitialConditions, 0.f);
		}
	}
}

void URealisticProjectileComponent::HandleInObjectTick(struct SubTickState& OutState, const struct SubTickState& InState, float GameTime, const UWorld* World, bool PredictOnly)
{
	// See if there are any exit hits on the way to new location
	FHitResult Exit;
	bool HadExitHit = GetExitHit(Exit, InState.Location, InState.Rotation, World);

	// Deal with hits
	if (MoveEntryHits.Num() > 0 || HadExitHit)
	{
		// We have to deal with a hit
		// But we only care about the first one encountered (closest one)
		float EntryDistance = MoveEntryHits.Num() > 0 ? (MoveEntryHits[0].Location - InState.Trajectory.InitialWorldLocation).SizeSquared() : INFINITY;
		float ExitDistance = HadExitHit ? (Exit.Location - InState.Trajectory.InitialWorldLocation).SizeSquared() : INFINITY;
		bool IsEntryHit = EntryDistance < ExitDistance;
		FHitResult HitToDealWith = IsEntryHit ? MoveEntryHits[0] : Exit;

#if WITH_EDITOR
		if (bDrawDebugHits && !PredictOnly)
		{
			FColor Col = HadExitHit ? FColor::Magenta : FColor::Emerald;
			DrawDebugSphere(World, HitToDealWith.Location, DebugHitSphereSize, 16, Col, false, DebugLineDuration);
		}
#endif

		// After a hit we'll still have more game time to use up. Update TickRemainingTime
		float TimeSinceLaunch = ComputeTimeOfFlight(InitialConditions, HitToDealWith.Location, Velocity);
		float TickRemainingTime = GameTime - (InitialConditions.t0 + TimeSinceLaunch);
		FVector NewVelocity = ComputeNewVelocity(TimeSinceLaunch);
		FRotator NewRotation = ComputeNewRotation(TimeSinceLaunch);

		// Now we can deal with the closest hit
		if (IsEntryHit)
		{
			// The new location should be nudged into the object slightly or it will
			// collide again and again every hit.
			FVector NewLocation = HitToDealWith.Location - HitToDealWith.ImpactNormal * NUDGE_DISTANCE;

			if (!PredictOnly)
			{
				//Tell blueprints about event happening
				OnProjectilePenetrate.Broadcast(HitToDealWith, Velocity);
				BroadcastHitOrOverlap(HitToDealWith);
			}

			// Add this hit to the list of objects currently penetrated
			ObjectsPenetrated.Emplace(HitToDealWith);
			FTrajectoryInitialConditions NewTrajectory = FTrajectoryInitialConditions(NewVelocity, NewLocation, InitialConditions.t0 + TimeSinceLaunch, NewRotation, GetNewPenetrationDeceleration(HitToDealWith));
			OutState = SubTickState(NewLocation, NewRotation, NewVelocity, NewTrajectory,TickRemainingTime);
		}
		else //HitToDealWith is an exit hit
		{
			FVector NewLocation = HitToDealWith.Location + HitToDealWith.ImpactNormal * NUDGE_DISTANCE;

			if (!PredictOnly)
			{
				OnProjectileAnyHit.Broadcast(HitToDealWith, Velocity);
				OnPenetrationExit.Broadcast(HitToDealWith, Velocity);
			}

			//Remove the matching entryhit
			ObjectsPenetrated.RemoveAll([&HitToDealWith](const FHitResult& Item) {return Item.GetComponent() == HitToDealWith.GetComponent(); });

			// Reset the initial conditions
			float Deceleration = 1.f;
			if (ObjectsPenetrated.Num() == 0)
			{
				NewVelocity = AdjustDirection(NewVelocity, HitToDealWith.ImpactNormal);
			}
			else {
				// TODO - instead of just using some object's deceleration - make them additive
				Deceleration = GetNewPenetrationDeceleration(ObjectsPenetrated[0]);
			}
			FTrajectoryInitialConditions NewTrajectory = FTrajectoryInitialConditions(NewVelocity, NewLocation, InitialConditions.t0 + TimeSinceLaunch, NewRotation, Deceleration);
			OutState = SubTickState(NewLocation, NewRotation, NewVelocity, NewTrajectory, TickRemainingTime);
		}
	}
	else // No hits
	{
		OutState = SubTickState(InState.Location, InState.Rotation, InState.Velocity, InState.Trajectory, 0.f);
	}
}

bool URealisticProjectileComponent::HasStoppedMoving(URealisticProjectileComponent* Self, bool PredictOnly)
{
	if (Self->UpdatedComponent == nullptr) { return true; }
	// FVector::IsNearlyZero is maybe too selective for these
	switch (Self->InitialConditions.TrajectoryType)
	{
		case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Air:
			return fabsf(Self->Settings.TerminalVelocity) < SMALL_NUMBER && Self->Velocity.SizeSquared() < 1.f;
		case FTrajectoryInitialConditions::ETrajectoryType::TT_In_Object:
			return Self->Velocity.SizeSquared() < 1.f;
		case FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding:
			if (Self->Settings.Friction <= 0.f)
			{
				return Self->InitialConditions.SlidingAcceleration.SizeSquared() < 1.f && Self->Velocity.SizeSquared() < 1.f;
			}
			else
			{
				if (PredictOnly) { return true; }
				float FrAccel = AccelerationDueToFriction(Self->Settings.Friction, Self->Settings.Gravity);
				return FrAccel * FrAccel > Self->InitialConditions.SlidingAcceleration.SizeSquared() && Self->Velocity.IsNearlyZero();
			}
		default:
			return true;
	}
}

bool URealisticProjectileComponent::SetupPredictedTrajectory(UObject* WorldContextObject, AActor*& OutActor, URealisticProjectileComponent*& OutComponent, UWorld*& OutWorld,
		const FVector& StartLocation, const FRotator& StartRotation, TSubclassOf<AActor> Properties, int32 RandSeed, bool bOverrideGravity, const FVector& GravityAccel, const FVector& Scale, 
		const FRealisticProjectileBehavior* Settings)
{
	// Get the world
	if (GEngine == nullptr || WorldContextObject == nullptr) { return false; }
	UWorld* World = GEngine->GetWorldFromContextObjectChecked(WorldContextObject);
	UClass* ActorType = Properties.Get();
	if (World == nullptr || ActorType == nullptr) 
	{
		//UE_LOG(ProjectilePhysics, Error, TEXT("Can't get world or input error"))
		return false;
	}

	FTransform SpawnTransform = FTransform(StartRotation, StartLocation, Scale);
	

	FActorSpawnParameters ASP;
	ASP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	ASP.bNoFail = true;
	OutActor = World->SpawnActor<AActor>(ActorType, SpawnTransform, ASP);
	if (!IsValid(OutActor))
	{
		//UE_LOG(ProjectilePhysics, Error, TEXT("Couldn't spawn actor"))
		return false;
	}

	URealisticProjectileComponent* SpawnedComponent = Cast<URealisticProjectileComponent>(OutActor->GetComponentByClass(URealisticProjectileComponent::StaticClass()));
	if (SpawnedComponent == nullptr)
	{
		//UE_LOG(ProjectilePhysics, Error, TEXT("Couldn't get spawned component"));
		OutActor->Destroy();
		return false; 
	}
	
	if (Settings)
	{ 
		SpawnedComponent->Settings = *Settings;
		SpawnedComponent->Velocity = Settings->InitialSpeed * StartRotation.Vector();
	}
	FVector GravToUse = bOverrideGravity ? GravityAccel : SpawnedComponent->Settings.Gravity;
	SpawnedComponent->k = SpawnedComponent->Settings.TerminalVelocity < SMALL_NUMBER ? 0.1f : fabsf(0.5f * GravToUse.Size() / SpawnedComponent->Settings.TerminalVelocity);
	SpawnedComponent->Vinf = GravToUse.GetSafeNormal() * SpawnedComponent->Settings.TerminalVelocity;
	SpawnedComponent->RandStream = FRandomStream(RandSeed);
	SpawnedComponent->Settings.Gravity = GravToUse;

	OutComponent = SpawnedComponent;
	OutWorld = World;
	return true;
}

void URealisticProjectileComponent::StartSliding(struct SubTickState& OutState, const struct SubTickState& InState, float GameTime, const FHitResult& Hit, float FlightTime, bool PredictOnly)
{
	FVector SlopeNormal = Hit.ImpactNormal;
	FVector SlideAccel = FVector::VectorPlaneProject(FVector(0, 0, GetGravityZ()), SlopeNormal);
	FVector NewVelocity = FVector::VectorPlaneProject(InState.Velocity, SlopeNormal);
	FVector AdjustedLocation = Hit.Location + (Hit.TraceStart - Hit.TraceEnd).GetSafeNormal() * SLIDING_HOVER_HEIGHT;
	TWeakObjectPtr<UPrimitiveComponent> Surface2 = TWeakObjectPtr<UPrimitiveComponent>();
	if (InitialConditions.TrajectoryType == FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding)
	{
		// If we were sliding already, we are possibly lodged between two surfaces.
		// If velocity will very soon head back into surface 1, and normals form a wedge opposing gravity
		FVector GNormal = Settings.Gravity.GetSafeNormal();
		if (InitialConditions.TrajectoryType == FTrajectoryInitialConditions::ETrajectoryType::TT_Sliding &&
			(NewVelocity + SlideAccel * .2f | InitialConditions.SlidingSurfaceNormal) < 0.f &&
			(FVector::VectorPlaneProject(Hit.ImpactNormal, GNormal) | FVector::VectorPlaneProject(InState.Trajectory.SlidingSurfaceNormal, GNormal)) <= 0.f)
		{
			Surface2 = InitialConditions.SlidingSurface;
			FVector MoveDirection = FVector::CrossProduct(Hit.ImpactNormal, InitialConditions.SlidingSurfaceNormal);
			if (MoveDirection.IsNearlyZero()) // Cross product can sometimes be 0 if inputs are parallel or perpendicular
			{ 
				MoveDirection = FVector::CrossProduct(Hit.ImpactNormal, Hit.ImpactNormal + InitialConditions.SlidingSurfaceNormal);
			}
			SlideAccel = FVector(0, 0, GetGravityZ()).ProjectOnTo(MoveDirection);
			NewVelocity = InState.Velocity.ProjectOnTo(MoveDirection);
			SlopeNormal = (Hit.ImpactNormal + InitialConditions.SlidingSurfaceNormal).GetUnsafeNormal();
			AdjustedLocation = Hit.Location + SlopeNormal * SLIDING_HOVER_HEIGHT;

			// If we were already sliding on 2 surfaces and got here, then there are 3 and we should stop
			if (InitialConditions.GetSlidingSurface2() != nullptr)
			{
				SlideAccel = FVector::ZeroVector;
				NewVelocity = FVector::ZeroVector;
				SlopeNormal = -Settings.Gravity.GetSafeNormal();
				StopSimulating(Hit, PredictOnly);
				OutState = SubTickState(AdjustedLocation, InState.Rotation, FVector::ZeroVector, InState.Trajectory, 0.f);
				return;
			}
		}
	}

	float TickRemainingTime = GameTime - (InState.Trajectory.t0 + FlightTime);
	FTrajectoryInitialConditions NewTrajectory = FTrajectoryInitialConditions(NewVelocity, AdjustedLocation, InState.Trajectory.t0 + FlightTime, InState.Rotation, SlideAccel, SlopeNormal, Hit.Component, Surface2);
	OutState = SubTickState(AdjustedLocation, InState.Rotation, NewVelocity, NewTrajectory, TickRemainingTime); 
}

void URealisticProjectileComponent::StopSliding(struct SubTickState& OutState, const struct SubTickState& InState, const FVector& StartTickLocation, const UWorld* World, float GameTime)
{
	UPrimitiveComponent* Surface = InState.Trajectory.SlidingSurface.Get(), *Surface2 = InState.Trajectory.SlidingSurface2.Get();
	const float CheckDepth = Surface2 == nullptr ? SLIDING_CHECK_DEPTH : 1.5f * SLIDING_CHECK_DEPTH; // How far downward to feel for floor
	FVector CheckLocation = InState.Location - InState.Trajectory.SlidingSurfaceNormal * CheckDepth;
	FComponentQueryParams CQP = FComponentQueryParams();
	CQP.bFindInitialOverlaps = false;
	// We can use one of the GetExitHit functions as it backtraces until finding a specific component
	bool Surf1Found, Surf2Found = false;
	FVector DropStartLocation = StartTickLocation - InState.Trajectory.SlidingSurfaceNormal * CheckDepth;

	FHitResult Hit;
	Surf1Found = GetExitHit(Hit, DropStartLocation, CheckLocation, InState.Rotation, World, Surface, CQP);
	if (Surface2 != nullptr)
	{
		FHitResult Hit2;
		Surf2Found = GetExitHit(Hit2, DropStartLocation, CheckLocation, InState.Rotation, World, Surface, CQP);
		if (Surf2Found && FVector::DistSquared(Hit2.Location, InState.Location) > FVector::DistSquared(Hit.Location, InState.Location))
		{
			Hit = Hit2;
		}
	}
	
	if (Surf1Found || Surf2Found) // Found a hit
	{
		FVector NewLocation = Hit.Location + InState.Trajectory.SlidingSurfaceNormal * CheckDepth;
		float FlightTime = ComputeTimeOfFlight(InState.Trajectory, Hit.Location, InState.Velocity);
		float TempTickRemainingTime = GameTime - (InState.Trajectory.t0 + FlightTime);
		FVector NewVelocity = ComputeNewVelocity(FlightTime);
		if (IsValid(UpdatedComponent)) // Set the angular velocity for spinning after rolling off ledge
		{
			FVector RotAxis = FVector(Velocity.Y, Velocity.X, -Velocity.Z).GetSafeNormal();
			float w = Velocity.Size() / (UpdatedComponent->Bounds.SphereRadius * 2 * PI);
			AngularVelocity = FQuat(RotAxis, w).Rotator();
		}

		FTrajectoryInitialConditions NewAirTrajectory = FTrajectoryInitialConditions(NewVelocity, NewLocation, GameTime, InState.Rotation);
		OutState = SubTickState(NewLocation, InState.Rotation, NewVelocity, NewAirTrajectory, TempTickRemainingTime);
	}
	else
	{
		// Error? I guess just make us an air projectile at the location given
		//UE_LOG(ProjectilePhysics, Error, TEXT("Error: Detected roll off ledge, but could not find exact edge"))
		FTrajectoryInitialConditions NewAirTrajectory = FTrajectoryInitialConditions(InState.Velocity, InState.Location, GameTime, InState.Rotation);
		OutState = SubTickState(InState.Location, InState.Rotation, InState.Velocity, NewAirTrajectory, 0.f);
	}
	
}

/** If conditions are met, turn collision off and attach to hit component,
* otherwise start simulating regular physics*/
void URealisticProjectileComponent::TryEmbed()
{
	if (ObjectsPenetrated.Num() == 0)
	{
		//UE_LOG(ProjectilePhysics, Error, TEXT("Bad state in RealisticProjectileComponent.cpp TryEmbed"))
		if (IsValid(GetOwner())) {GetOwner()->Destroy();}
		return;
	}

	// Choose the component first entered so we don't bounce out of something inside another object
	FHitResult OutermostHit = ObjectsPenetrated[0];
	// Remember when we started the trajectory, the component was nudged into the object from hit location
	FVector ActualInitialLocation = OutermostHit.Location - OutermostHit.ImpactNormal * NUDGE_DISTANCE;
	FVector CurrentLocation = ComputeNewLocation(InitialConditions.EndTrajectoryTime);

	UPrimitiveComponent* HitComponent = OutermostHit.GetComponent();
	if (!IsValid(UpdatedComponent) || !IsValid(UpdatedPrimitive) || !IsValid(HitComponent))
	{
		//UE_LOG(ProjectilePhysics, Error, TEXT("Bad state in RealisticProjectileComponent.cpp TryEmbed"))
		if (IsValid(GetOwner())) { GetOwner()->Destroy(); }
		return;
	}

	float PenDistance = (CurrentLocation - ActualInitialLocation).Size();
	// Get the collision velocity. Since we maybe forgot the velocity of the outermost hit
	// we must calculate it using how far in the object we are.
	// eq(1) v = at + v0
	// set v=0 for velocity at rest-time and rearrange
	// eq(2) t = -v0/a
	// eq(3) p = 0.5at^2 + v0.t
	// Substitute eq(2) into eq(3) for position at rest time
	// p = 0.5a(v0^2/a^2) + v0(-v0/a)
	// p = 0.5v0^2/a - v0^2/a
	// p = -0.5v0^2/a
	// 
	// Rearrange to find v0 (initial velocity) given a position
	// v0 = sqrt(-2.a.p) where a is a -ve number
	float ImpactSpeed = sqrtf(-2.f * GetNewPenetrationDeceleration(OutermostHit) * PenDistance);
	FVector ImpactVelocity = ImpactSpeed * (OutermostHit.TraceEnd - OutermostHit.TraceStart).GetSafeNormal();
	//UE_LOG(ProjectilePhysics, Display, TEXT("Pendist=%f ImpactSpd=%f flighttime=%f"), PenDistance, ImpactSpeed, InitialConditions.EndTrajectoryTime)

	float MinimumEmbedDepth = -1.f;
	if (Settings.MaximumEmbedToughness > 0.f)
	{
		// Work out if this satisfies the condition for maximum embed toughness. The best way without introducing an inverse to the Toughness->Deceleration function (which
		// would make changing the math fraught with peril), is to get the penetration distance of the max toughness and compare with ours.
		float MaxToughnessDeceleration = InternalGetPenetrationDeceleration(Settings.MaximumEmbedToughness, Settings.DecelerationExponent, Settings.DecelerationCoefficient);
		float MinimumEmbedSpeed = 0.95f * Settings.InitialSpeed; // Below 95% initial speed, projectile will no longer penetrate
		MinimumEmbedDepth = -0.5f * MinimumEmbedSpeed * MinimumEmbedSpeed / MaxToughnessDeceleration; // Use the formula position at rest time from last code block
	}

	if (PenDistance > MinimumEmbedDepth)
	{
		// Embed projectile
		
		OnProjectileEmbed.Broadcast(OutermostHit, ImpactSpeed);
		UpdatedPrimitive->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		UpdatedComponent->AttachToComponent(HitComponent, FAttachmentTransformRules(EAttachmentRule::KeepWorld, true), OutermostHit.BoneName);
		ApplyImpulse(OutermostHit, Velocity);
	}
	else
	{
		// Bounce projectile and set simulate physics
		float VelocityIntoSurface = -(ImpactVelocity | OutermostHit.ImpactNormal);
		float OneMinusBounciness = 1.f - Settings.Bounciness;
		Velocity = ImpactVelocity.MirrorByVector(OutermostHit.ImpactNormal) - (OneMinusBounciness * VelocityIntoSurface * OutermostHit.ImpactNormal);
		Velocity = AdjustDirection(Velocity, OutermostHit.ImpactNormal);

		ApplyImpulse(OutermostHit, ImpactVelocity * OneMinusBounciness);
		FVector WorldLocation = OutermostHit.Location + OutermostHit.ImpactNormal  * NUDGE_DISTANCE * 25.f;
		FRotator Rot = UpdatedPrimitive->GetComponentRotation();
		Settings.bRotationFollowsVelocity = false;
		UpdatedPrimitive->SetWorldLocationAndRotation(WorldLocation, Rot);

		UpdatedPrimitive->SetSimulatePhysics(true);
		UpdatedPrimitive->SetPhysicsLinearVelocity(Velocity, false);
		UpdateComponentVelocity();
		// Add a small rotation impulse to make arrows unlikely to stand on end for example.
		FVector NudgeAngular = FVector(RandStream.FRandRange(-0.001, 0.001f), RandStream.FRandRange(-0.001, 0.001f), 0.f);
		if (IsValid(UpdatedPrimitive)) { UpdatedPrimitive->SetAllPhysicsAngularVelocityInRadians(NudgeAngular); }
	
		AActor* Owner = GetOwner();
		if (IsValid(Owner))
		{
			for (const UActorComponent* Comp : Owner->GetComponents())
			{
				if (Comp->IsA(UPrimitiveComponent::StaticClass()))
				{
					((UPrimitiveComponent*)Comp)->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
				}
			}
		}
		//UE_LOG(ProjectilePhysics, Display, TEXT("Arrow bounced out of object. ImpactV=%f"), ImpactSpeed)
		StopSimulating(OutermostHit);

		OnProjectileBounce.Broadcast(OutermostHit, Velocity);
	}
}

#if WITH_EDITOR
bool URealisticProjectileComponent::CanEditChange(const UProperty* InProperty) const
{
	const bool ParentVal = Super::CanEditChange(InProperty);

	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(URealisticProjectileComponent, bDebugLineColorFromVelocity))
	{
		return bDrawDebugLine;
	}

	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(URealisticProjectileComponent, DebugLineThickness))
	{
		return bDrawDebugLine;
	}

	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(URealisticProjectileComponent, DebugLineDuration))
	{
		return bDrawDebugLine || bDrawDebugHits;
	}

	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(URealisticProjectileComponent, DebugLineAuxiliaryColor))
	{
		return bDrawDebugLine && !bDebugLineColorFromVelocity;
	}

	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(URealisticProjectileComponent, DebugHitSphereSize))
	{
		return bDrawDebugHits;
	}

	return ParentVal;
}
#endif // WITH_EDITOR