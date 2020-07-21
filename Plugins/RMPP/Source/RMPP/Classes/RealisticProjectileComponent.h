// Copyright 2018 Mipmap Games.

#pragma once

#include "RMPP.h"
#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "UObject/Object.h"
#include "Engine/DataTable.h"
#include "GameFramework/MovementComponent.h"
#include "RealisticProjectileComponent.generated.h"

class UPrimitiveComponent;
struct FComponentQueryParams;

/** A Data structure for extra information we want to ascribe to physical materials.
* At the moment just toughness. */
USTRUCT(BlueprintType)
struct FMaterialProperties : public FTableRowBase
{
	GENERATED_USTRUCT_BODY()

public:

	FMaterialProperties()
		: Toughness(1.f)
	{}

	/** A value that affects how likely a ricochet is and how much this material slows projectiles down. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = MaterialProperty)
	float Toughness;
};

/** Each tick the component calculates a new location based on the initial conditions,
* NOT based on the location or velocity last tick. After hitting an object, new initial
* conditions should be created with a new trajectory. */
USTRUCT(BlueprintType)
struct FTrajectoryInitialConditions
{
	GENERATED_USTRUCT_BODY()

	enum class ETrajectoryType : uint8
	{
		TT_In_Air,
		TT_In_Object,
		TT_Sliding
	};

	/** Whether it's in air, obj or sliding */
	ETrajectoryType TrajectoryType;

	/** Velocity of actor on spawn, or after bounce, penetration or exit*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Trajectory)
	FVector InitialVelocity;

	/** World location of the actor on spawn, or after bounce, penetration, or exit*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Trajectory)
	FVector InitialWorldLocation;

	/** Orientation at launch time. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Trajectory)
	FRotator InitialRotation;

	/** Game time the current trajectory began */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Trajectory)
	float t0;

	/** Unit vector in the direction of launch velocity. */
	UPROPERTY()
	FVector v0direction;

	/** The size of velocity vector at launch. */
	UPROPERTY()
	float v0length;

	/** Deceleration experienced inside an object. Should be -ve, a value of 1.0 means we are in air. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Trajectory)
	float PenDeceleration;

	/** Time in seconds after launch the projectile will stop moving. */
	UPROPERTY()
	float EndTrajectoryTime;

	/** Portion of gravitational acceleration in effect while sliding. cm/s/s in each dimension. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Trajectory)
	FVector SlidingAcceleration = FVector::ZeroVector;

	/** Normal vector pointing out from the surface we're sliding on. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Trajectory)
	FVector SlidingSurfaceNormal = FVector::ZeroVector;

	/** The component we are sliding along */
	TWeakObjectPtr<class UPrimitiveComponent> SlidingSurface = TWeakObjectPtr<class UPrimitiveComponent>();

	/** Another component we are sliding along (could be wedged against two surfaces). Could be nullptr */
	TWeakObjectPtr<class UPrimitiveComponent> SlidingSurface2 = TWeakObjectPtr<class UPrimitiveComponent>();

	/** The surface we are sliding on. Could return nullptr. */
	UPrimitiveComponent* GetSlidingSurface()
	{
		return SlidingSurface.Get();
	}

	/** The other surface we are sliding on (wedged between). Could return nullptr. */
	UPrimitiveComponent* GetSlidingSurface2()
	{
		return SlidingSurface2.Get();
	}

	FTrajectoryInitialConditions()
	{}

	/**
	* @param  v0				Initial Velocity
	* @param  Loc0				Initial Location
	* @param  GameTime			Gametime at launch
	* @param  Rotation			Rotation at launch
	* @param  PenetrationDeceleration	Amount of deceleration experienced in the object. If not needed give +ve value.
	*/
	FTrajectoryInitialConditions(FVector v0, FVector Loc0, float GameTime, FRotator Rotation, float PenetrationDeceleration = 1.f)
	{
		TrajectoryType = 0.f < PenetrationDeceleration ? ETrajectoryType::TT_In_Air : ETrajectoryType::TT_In_Object;
		InitialVelocity = v0;
		InitialWorldLocation = Loc0;
		InitialRotation = Rotation;
		t0 = GameTime;
		v0.ToDirectionAndLength(v0direction, v0length);
		PenDeceleration = PenetrationDeceleration;
		EndTrajectoryTime = v0length / -PenetrationDeceleration;
		//UE_LOG(LogTemp, Warning, TEXT("New Initial Conditions:Spd=%f, Decel=%f t0=%f"), v0.Size(), PenetrationDeceleration, GameTime)
	}

	/**
	 * @param  v0				Initial Velocity
	 * @param  Loc0				Initial Location
	 * @param  GameTime			Gametime at launch
	 * @param  Rotation			Rotation at launch
	 * @param  SlideAcceleration Accel due to gravity on this slope.
	 * @param  SlideNormal		The normal of the surface hit
	 * @param  SlideComponent	The component we slide along
	 * @param  SlideComponent2	Another component we slide along (wedged)
	 */
	FTrajectoryInitialConditions(FVector v0, FVector Loc0, float GameTime, FRotator Rotation, FVector SlideAcceleration, FVector SlideNormal,
		TWeakObjectPtr<UPrimitiveComponent> SlideComponent, TWeakObjectPtr<UPrimitiveComponent> SlideComponent2 = TWeakObjectPtr<UPrimitiveComponent>())
	{
		TrajectoryType = ETrajectoryType::TT_Sliding;
		InitialVelocity = v0;
		InitialWorldLocation = Loc0;
		InitialRotation = Rotation;
		t0 = GameTime;
		v0.ToDirectionAndLength(v0direction, v0length);
		SlidingAcceleration = SlideAcceleration;
		SlidingSurfaceNormal = SlideNormal;
		SlidingSurface = SlideComponent;
		SlidingSurface2 = SlideComponent2;
		//UE_LOG(LogTemp, Warning, TEXT("New Initial Conditions Sliding:Spd=%f, t0=%f"), v0.Size(), GameTime)
	}
};

/**
 * All the properties that define how a projectile will act in any given situation.
 */
USTRUCT(BlueprintType)
struct FRealisticProjectileBehavior
{
	GENERATED_USTRUCT_BODY()

	/** Initial speed of projectile on spawn in cm/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float InitialSpeed = 65000.f;

	/** Terminal velocity of the projectile when falling. This essentially decides the drag characteristics
	* of the projectile. cm/s. Generally should be positive. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float TerminalVelocity = 9000.f;
	
	/** Force of gravity in cm/s/s */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	FVector Gravity = FVector(0.f, 0.f, -980.f);

	/**
	* Projectile will not become embedded in objects tougher than this. Good for making sure arrows
	* can't stick in stone. If object is too tough to embed, collision of all components is turned on
	* and simulate physics is set true on root component. The calculation is affected by impact velocity, with the way it is implemented
	* it would be more accurately called 'Maximum Embed Toughness At 95% Initial Speed'. A negative value
	* turns this feature off. The default toughness is 1000
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float MaximumEmbedToughness = -1.f;

	/* If true, this projectile will have its rotation updated each frame to match the direction of its velocity. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	uint32 bRotationFollowsVelocity : 1;

	/* The minimum speed of the projectile required to penetrate a material (head on) - when speed is divided by material toughness.
	 * For example encountering a material with 4000 toughness, with this value at 10 the minimum speed is 40 000cm/s */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float MinPenetrationVelocityFactor = 30.f;

	/* When (speed/toughness) is greater than this factor, projectile will always penetrate - unless the angle of impact prevents it.
	 * For example encountering a material with 1000 toughness, with this value at 50 a projectile will always penetrate at > 50000cm/s */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float AlwaysPenetrateVelocityFactor = 40.f;

	/* The maximum angle in radians from the perpendicular that a projectile can still penetrate. 0=head on, 1.57=Completely parallel */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float AlwaysRicochetAngle = 1.45f;

	/* The angle in radians from the perpendicular below which a projectile will always penetrate - unless velocity prevents it. 0=head on, 1.57=Completely parallel */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float AlwaysPenetrateAngle = 0.5f;

	/* 
	* Effects the deceleration experienced when inside other objects. Deceleration is calculated as -at^e cm/s/s where
	* a = DecelerationCoefficient
	* t = Toughness (of material)
	* e = DecelerationExponent
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float DecelerationCoefficient = 2.f;

	/* 
	* Deceleration while inside objects is calculated as -at^e cm/s/s where
	* a = DecelerationCoefficient
	* t = Toughness (of material)
	* e = DecelerationExponent
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float DecelerationExponent = 2.55f;

	/*
	* Amount of randomness added when projectile exits a material
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float ExitRandomness = 2.f;

	/**
	* Percentage of velocity maintained after the bounce in the direction of the normal of impact (coefficient of restitution).
	* 1.0 = no velocity lost, 0.0 = no bounce.
	*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileBounce, meta = (ClampMin = "0", UIMin = "0"))
	float Bounciness = 0.1f;

	/* If the updated component should act like a rolling ball when skidding along the ground/a surface. If it looks crappy, turn motion blur
	 * off in project settings. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileSliding)
	bool bRollsWhenSliding = true;

	/* When sliding, a friction generally from 0-1. 0 or lower will be frictionless. This is not air drag. Use TerminalVelocity to control drag. Sliding
	 * with friction is not deterministic. Without friction is. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileSliding)
	float Friction = -1.f;


};

/* This class moves an actor component (->RootComponent by default) and supports penetration,
* ricochet, drag and embedding. The projectile moves in a deterministic way,
* so on different machines, the path of this object will remain exactly the same.
* When used for network play the projectile should use blocking collision
* with static objects but overlap with dynamic objects (players, physobj). */
UCLASS(ClassGroup = Movement, meta = (BlueprintSpawnableComponent), ShowCategories = (Velocity))
class URealisticProjectileComponent : public UMovementComponent
{
	GENERATED_UCLASS_BODY()

	DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnProjectileAnyHitDelegate, const FHitResult&, HitDetails, const FVector&, ImpactVelocity);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnProjectileBounceDelegate, const FHitResult&, ImpactResult, const FVector&, ImpactVelocity);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnProjectilePenetrateDelegate, const FHitResult&, ImpactResult, const FVector&, ImpactVelocity);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnPenetrationExitDelegate, const FHitResult&, ExitHit, const FVector&, ExitVelocity);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnProjectileStopDelegate, const FHitResult&, ImpactResult);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnProjectileEmbedDelegate, const FHitResult&, HitResult, float, ImpactVelocity);
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnComponentBeginOverlapDelegate, const FHitResult&, HitResult, int32, TrajectoryNum, float, TimeSinceSpawn);

private:
	/* Distance to nudge projectile into or out of objects after hit (to prevent collision on the following tick) */
	static constexpr float NUDGE_DISTANCE = 0.2f;
	/* How far off the ground to keep sliding objects. Too small and floating point errors may cause collisions with the floor it is sliding on */
	static constexpr float SLIDING_HOVER_HEIGHT = 1.1f;
	/* How far to sweep down to feel floor when sliding */
	static constexpr float SLIDING_CHECK_DEPTH = 5.f;
	/* Hit an object slower than this and projectile will slide. */
	static constexpr float START_SLIDING_VELOCITY = 80.f;
	static constexpr float DEFAULT_TOUGHNESS = 1000.f;

public:
#if WITH_DEV_AUTOMATION_TESTS
	friend class FRealisticProjectileComponentTest;
#endif

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	FRealisticProjectileBehavior Settings;

	/* Initial Angular velocity in degrees/second, a random value between min and max will be chosen. Good for spinning arrows */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Projectile Behavior")
	FRotator AngularVelocityMin = FRotator(0.f, 0.f, 360.f);

	/* Initial Angular velocity in degrees/second, a random value between min and max will be chosen. Good for spinning arrows */
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Projectile Behavior")
	FRotator AngularVelocityMax = FRotator(0.f, 0.f, 900.f);

	/* Turn on to draw trails behind projectiles. Can not be included in packaged game. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileDebugging)
	uint8 bDrawDebugLine : 1;

	/**
	 * When enabled debug line color changes depending on how fast the projectile is going with respect to InitialSpeed.
	 * When disabled, DebugLineAuxiliaryColor is used.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileDebugging)
	uint8 bDebugLineColorFromVelocity : 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileDebugging)
	float DebugLineThickness = 0.f;

	/* How long the debug line should show in the world in seconds. Also controls debug hit spheres */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileDebugging)
	float DebugLineDuration = 10.f;

	/* Color to draw when DebugLineColorFromVelocity is off */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileDebugging)
	FColor DebugLineAuxiliaryColor = FColor(255, 255, 0);

	/* Draw spheres at hit locations and exit locations. Duration is same as lines */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileDebugging)
	uint8 bDrawDebugHits : 1;

	/* Size of spheres drawn at hits and exits */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ProjectileDebugging)
	float DebugHitSphereSize = 4.f;

	/** 
	* The path to the datatable asset which stores extra information about
	* physical materials. At the moment this is just Toughness.
	*/
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Physical Materials")
	FString DataTablePath = TEXT("/RMPP/DataTables/TBL_MaterialPropertiesTable");
	
	/* Multiplies the amount of momentum applied to hit physics objects */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Projectile Behavior")
	float ImpulseScale = 1.f;

	// Begin Delegates

	/** Called on any hit. Other events like bounce, penetrate, embed will be called after*/
	UPROPERTY(BlueprintAssignable)
	FOnProjectileAnyHitDelegate OnProjectileAnyHit;

	/** Called when projectile impacts something and bounces are enabled. */
	UPROPERTY(BlueprintAssignable)
	FOnProjectileBounceDelegate OnProjectileBounce;

	/** Called when projectile penetrates an object. */
	UPROPERTY(BlueprintAssignable)
	FOnProjectilePenetrateDelegate OnProjectilePenetrate;

	/** Called when projectile exits an object. */
	UPROPERTY(BlueprintAssignable)
	FOnPenetrationExitDelegate OnPenetrationExit;

	/** Called when projectile has come to a stop (velocity is below simulation threshold, bounces are disabled, or it is forcibly stopped). */
	UPROPERTY(BlueprintAssignable)
	FOnProjectileStopDelegate OnProjectileStop;

	/** Called when projectile becomes embedded in an object. Returns false if an arrow 'glances' off object*/
	UPROPERTY(BlueprintAssignable)
	FOnProjectileEmbedDelegate OnProjectileEmbed;

	/** Called when projectile overlaps something - generally a player pawn*/
	UPROPERTY(BlueprintAssignable)
	FOnComponentBeginOverlapDelegate OnComponentBeginOverlap;

	// Begin state UProperties

	/** The stream to get randomness from. If the same seed is used, random numbers are same across machines */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "State|AntiCheat")
	FRandomStream RandStream;

	/** Not used yet - but would be a good way to prevent cheating : did the projectile follow the same path on server machine? */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "State|AntiCheat")
	TArray<FTrajectoryInitialConditions> InitialConditionHistory;

	/** Angular velocity in degrees/second */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "State")
	FRotator AngularVelocity;

	/**
	* Every time a blocking hit is encountered, the HitResult is added to this array.
	* When component is exited, the matching entry hit is found and removed from the array.
	* Entries should be matched by components, not actors.
	*/
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "State")
	TArray<struct FHitResult> ObjectsPenetrated;

	// Begin UFunctions

	/**
	 * Given the game time, compute a new location.
	 * @param  TimeSinceLaunch			Game time (affected by pause/dilate) since trajectory start
	 * @return World Location at new time.
	 */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	virtual FVector ComputeNewLocation(float TimeSinceLaunch);

	/**
	* Given the game time, compute a new Rotation
	* @param  TimeSinceLaunch			Game time (affected by pause/dilate) since trajectory start
	* @return World Rotation at new time.
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	virtual FRotator ComputeNewRotation(float TimeSinceLaunch);

	/**
	* Given the game time, compute a new velocity.
	* @param  TimeSinceLaunch			Game time (affected by pause/dilate) since trajectory start
	* @return World velocity at new time.
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	virtual FVector ComputeNewVelocity(float TimeSinceLaunch);

	/**
	* Given an array of StaticMeshComponents, change their world location and scale to make a visible trajectory in the world.
	* If the trajectory stops short eg only uses a few meshes, the remaining meshes will have visibility set to false. Also see
	* GetPredictedPath.
	* @param	Meshes				An array of StaticMeshComponents that will be translated and scaled. They should all be the same mesh as the first one. They should
	*								be 1 unit/cm long, point in the X direction and start at the origin ie. no vertices in -X
	* @param	StartLocation		The initial location of the trajectory
	* @param	StartRotation		The initial rotation of the trajectory
	* @param	ProjectileClass		An Actor subclass that should contain a ProjectileMovemenComponent
	* @param	RandSeed			A random seed to determine some hits/bounces and direction after hit.
	* @param	TimeToSimulate		Stop planning the trajectory after the imaginary projectile would have travelled for this long (seconds)
	* @param	StepTime			Simulate a new tick every _ seconds
	* @param	OverrideGravity		Whether the next parameter, GravityAccel, has any effect
	* @param	GravityAccel		The force of gravity in cm/s/s - currently only z axis is supported
	* @param	Thickness			Scale the Y and Z dimensions of the meshes up or down (thinner/thicker trace)
	* @return An array of World Location vectors.
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile", meta = (WorldContext = "WorldContextObject"))
	static void DrawPredictedPath(UObject* WorldContextObject, TArray<UStaticMeshComponent*> Meshes, FVector StartLocation, FRotator StartRotation,
		TSubclassOf<AActor> ProjectileClass, int32 RandSeed = 0, float TimeToSimulate = 3.f, float StepTime = 0.1f, bool bOverrideGravity = false,
		FVector GravityAccel = FVector(0.f, 0.f, -980.f), float Thickness = 1.f);

	/**
	* Get a blocking hit with the first exit location the projectile will encounter (from stored entry hits to next location).
	* @param  OutHit			The first exit hit encountered will be stored here - if any.
	* @param  NewLocation		The location of the projectile at the end of this tick
	* @param  Rotation			Rotation for sweeping
	* @param  World				The world object
	* @return true if a hit was found. The hit will be stored in outHit
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	virtual bool GetExitHit(FHitResult &OutHit, FVector NewLocation, FRotator Rotation, const UWorld* World);

	/** Get the information that defines current trajectory */
	UFUNCTION(BlueprintPure, Category = "Game|Components|RealisticProjectile")
	virtual FTrajectoryInitialConditions GetInitialConditions() { return InitialConditions; }
	
	/**
	* Get the expected trajectory of a projectile with the given random seed and data.
	* @param	StartLocation		The initial location of the trajectory
	* @param	StartRotation		The initial rotation of the trajectory
	* @param	Properties			A class with the behaviour of the projectile eg drag, start speed, penetration parameters
	* @param	RandSeed			A random seed to determine some hits/bounces and direction after hit.
	* @param	TimeToSimulate		Stop planning the trajectory after the imaginary projectile would have travelled for this long (seconds)
	* @param	StepTime			Simulate a new tick every _ seconds
	* @param	MaxPoints			The maximum size of the array to return. If the projectile hits many things, the points returned will be greater than TimeToSimulate/StepTime
	* @param	OverrideGravity		Whether the next parameter, GravityAccel, has any effect
	* @param	GravityAccel		Use this gravity for the prediction. Currently only Z axis supported
	* @param	Scale				Scale to use for the prediction actor
	* @return An array of World Location vectors, and the velocity at the end of simulation.
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile", meta = (WorldContext = "WorldContextObject"))
	static void GetPredictedTrajectory(UObject* WorldContextObject, TArray<FVector>& WorldLocations, FVector& LastVelocity, FVector StartLocation, FRotator StartRotation, TSubclassOf<AActor> Properties, 
		int32 RandSeed = 0, float TimeToSimulate = 3.f, float StepTime = 0.1f, int32 MaxPoints = 400, 
		bool bOverrideGravity = false, FVector GravityAccel = FVector(0.f, 0.f, -980.f), FVector Scale = FVector(1.f, 1.f, 1.f));

	/**
	* Get the expected trajectory of a projectile with the given random seed and data.
	* @param	StartLocation		The initial location of the trajectory
	* @param	StartRotation		The initial rotation of the trajectory
	* @param	Properties			A class with the behaviour of the projectile eg drag, start speed, penetration parameters
	* @param	RandSeed			A random seed to determine some hits/bounces and direction after hit.
	* @param	TimeToSimulate		Stop planning the trajectory after the imaginary projectile would have travelled for this long (seconds)
	* @param	StepTime			Simulate a new tick every _ seconds
	* @param	MaxPoints			The maximum size of the array to return. If the projectile hits many things, the points returned will be greater than TimeToSimulate/StepTime
	* @param	OverrideGravity		Whether the next parameter, GravityAccel, has any effect
	* @param	GravityAccel		Use this gravity for the prediction. Currently only Z axis supported
	* @param	Scale				Scale to use for the prediction actor
	* @return An array of World Locations, rotations and velocities
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile", meta = (WorldContext = "WorldContextObject"))
	static void GetPredictedTrajectoryFull(UObject* WorldContextObject, TArray<FVector>& WorldLocations, TArray<FRotator>& Rotations, TArray<FVector>& Velocities, 
		FVector StartLocation, FRotator StartRotation, TSubclassOf<AActor> Properties, int32 RandSeed = 0, float TimeToSimulate = 3.f, float StepTime = 0.1f, 
		int32 MaxPoints = 400, bool bOverrideGravity = false, FVector GravityAccel = FVector(0.f, 0.f, -980.f), FVector Scale = FVector(1.f, 1.f, 1.f));

	/**
	* Get the expected trajectory of a projectile with the given random seed, data, and custom settings for a projectile.
	* @param	StartLocation		The initial location of the trajectory
	* @param	StartRotation		The initial rotation of the trajectory
	* @param	Properties			A class with the behaviour of the projectile eg drag, start speed, penetration parameters
	* @param	RandSeed			A random seed to determine some hits/bounces and direction after hit.
	* @param	TimeToSimulate		Stop planning the trajectory after the imaginary projectile would have travelled for this long (seconds)
	* @param	StepTime			Simulate a new tick every _ seconds
	* @param	MaxPoints			The maximum size of the array to return. If the projectile hits many things, the points returned will be greater than TimeToSimulate/StepTime
	* @param	OverrideGravity		Whether the next parameter, GravityAccel, has any effect
	* @param	GravityAccel		Use this gravity for the prediction. Currently only Z axis supported
	* @param	Scale				Scale to use for the prediction actor
	* @return An array of World Locations, rotations and velocities
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile", meta = (WorldContext = "WorldContextObject"))
	static void GetPredictedTrajectoryForSettings(UObject* WorldContextObject, TArray<FVector>& WorldLocations, FVector StartLocation, FRotator StartRotation, 
		TSubclassOf<AActor> Properties, FRealisticProjectileBehavior NewSettings, int32 RandSeed = 0, float TimeToSimulate = 3.f, float StepTime = 0.05f, int32 MaxPoints = 200,
		bool bOverrideGravity = false, FVector GravityAccel = FVector(0.f, 0.f, -980.f), FVector Scale = FVector(1,1,1));

	/**
	* Since things disappear in the distance, they must be scaled up to remain visible. Translates and scales a mesh to be visible.
	* @param	Mesh				The mesh to scale and translate. Should sit on top of the YZ plane and point in the X direction, and be 1 unit/cm long.
	* @param	ProjectileComponent	Initial conditions are used to ensure tracer does not clip through start point
	* @param	WorldLocation		Mesh will be sent here
	* @param	Velocity			Current Velocity
	* @param	CameraLocation		Mesh will be scaled in YZ plane based on how far this is
	* @param	Thickness			Overall scale up/down the YZ size of mesh.
	* @param	LengthFactor		How much to proportionally scale from velocity
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	static void ScaleTracerMesh(UStaticMeshComponent* Mesh, const URealisticProjectileComponent* ProjectileComponent, const FVector& WorldLocation, 
		const FVector& CameraLocation, float Thickness = 1.f, float LengthFactor = 0.06f);

	/**
	* Since things disappear in the distance, they must be scaled up to remain visible. Translates and scales a mesh to a long thin mesh pointing in Direction.
	* @param	Mesh			The mesh to scale and translate. Should sit on top of the YZ plane and point in the X direction, and be 1 unit/cm long.
	* @param	WorldLocation	Mesh will be sent here
	* @param	Direction		Mesh will be scaled this long and rotated this direction
	* @param	CameraLocation	Mesh will be scaled in YZ plane based on how far this is
	* @param	Thickness		Overall scale up/down the YZ size of mesh.
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	static void ScaleVisualEffect(UStaticMeshComponent* Mesh, const FVector& WorldLocation, const FVector& Direction, const FVector& CameraLocation, float Thickness);

	/** Change the current trajectory */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	virtual void SetInitialConditions(FTrajectoryInitialConditions Trajectory);
	
	/** Sets the velocity to the new value, rotated into Actor space. */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	virtual void SetVelocityInLocalSpace(FVector NewVelocity);
	
	/**
	* Decide whether to bounce or penetrate material based on the currenty velocity, material, and angle of impact.
	* @param  Hit				The hit with impact normal and also material information
	* @param  ImpactVelocity	Yep
	* @return true to bounce, false to penetrate. */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	bool ShouldBounce(const FHitResult& Hit, const FVector& ImpactVelocity);

	/** Clears the reference to UpdatedComponent, fires stop event (OnProjectileStop), and stops ticking (if bAutoUpdateTickRegistration is true). */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile")
	virtual void StopSimulating(const FHitResult& HitResult, bool PredictOnly = false);

	// public static functions
	/**
	* After receiving message from client saying they hit a player, check they did not cheat - that is, the bullet goes where they said it went, and the hit
	* player is within a reasonable distance.
	* @param	ActiveProjectiles		All the projectiles currently in the world
	* @param	Hit						The hit
	* @param	TimeSinceShot			The gametime elapsed from shooting the bullet to hitting the actor.
	* @param	BulletRandSeed			The random seed of the bullet. Together with shoot time, this should identify a bullet
	* @param	TrajectoryNumber		0-based index of trajectory history of this hit. Essentially number of ricochets/penetrations.
	* @param	Shooter					The player who dealt damage.
	* @param	Epsilon					The acceptable distance of the hit to where the hit player currently is (cm)
	* @param	StepTime				Step time to re-create trajectory with
	*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|RealisticProjectile|AntiCheat", meta = (WorldContext = "WorldContextObject"))
	static bool ServerCheckClientHit(UObject* WorldContextObject, FVector& ImpactVelocity, TArray<AActor*> ActiveProjectiles, FHitResult Hit, float TimeSinceShot, int32 BulletRandSeed, 
		float Epsilon = 100.f, float StepTime = 0.1f, bool DrawHit = false, bool bOverrideGravity = false, FVector NewGravity = FVector(0, 0, -980.f));

	// State calculation functions
	/**
	* Calculate position at time as per carpentier paper.
	* p(t) = ((v0 + kt Vinf) t) / (1 + kt)     +    p0
	* @param  TIC				The initial conditions (Start location p0, start Velocity v0)
	* @param  k				k = 0.5 * Gravity / TerminalVelocity. A drag factor usually from 0-0.5.
	* @param  Vinf				Vinf = TerminalVelocity + Wind. No wind currently.
	* @param  TimeSinceLaunch	The game time since t0 (in TIC.t0)
	*/
	UFUNCTION(BlueprintPure, Category = "Game|Components|RealisticProjectile")
	static FVector PositionAtTime_LinearDrag(const FTrajectoryInitialConditions& TIC, float k, const FVector& Vinf, float TimeSinceLaunch);

	/**
	* Calculate the time it took to get from launch to 'EndLocation' as per carpentier paper.
	* @param  TIC				The initial conditions (Start location p0, start Velocity v0)
	* @param  k					k = 0.5 * Gravity / TerminalVelocity. A drag factor usually from 0-0.5.
	* @param  Vinf				Vinf = TerminalVelocity + Wind. No wind currently.
	* @param  EndLocation		The position to use to calculate time of flight.
	* @param  CurrentVelocity	The current velocity can decide whether the trajectory is before or after apex
	*/
	UFUNCTION(BlueprintPure, Category = "Game|Components|RealisticProjectile")
	static float TimeOfFlight_LinearDrag(const FTrajectoryInitialConditions& TIC, float k, const FVector& Vinf, const FVector& EndLocation, const FVector& CurrentVelocity);
	
	/**
	* Calculate velocity at time as per carpentier paper.
	* (v0 + 2 k^2 t^2 Vinf) / (1 + kt)^2
	* @param  TIC				The initial conditions (Start location p0, start Velocity v0)
	* @param  k					k = 0.5 * Gravity / TerminalVelocity. A drag factor usually from 0-0.5.
	* @param  Vinf				Vinf = TerminalVelocity + Wind. No wind currently.
	* @param  TimeSinceLaunch	The game time since t0 (in TIC.t0)
	*/
	UFUNCTION(BlueprintPure, Category = "Game|Components|RealisticProjectile")
	static FVector VelocityAtTime_LinearDrag(const FTrajectoryInitialConditions& TIC, float k, const FVector& Vinf, float TimeSinceLaunch);

	/**
	* Calculate the time it took to get from initial to 'EndLocation' when sliding. (Solve quadratic)
	* @param	TIC					The initial conditions of the slide including accel due to gravity, location, velocity
	* @param	EndLocation			Caclulate the time from initial location to here.
	* @param	CurrentVelocity		We may need the current velocity to tell whether projectile is before its apex or after
	* @return Gametime from the launch time (TIC.t0) to when the object slid to EndLocation.
	*/
	UFUNCTION(BlueprintPure, Category = "Game|Components|RealisticProjectile")
	static float TimeOfFlight_Sliding(const FTrajectoryInitialConditions& TIC, const FVector& EndLocation, const FVector& CurrentVelocity);

	// public non-UFunctions

	/** This will check to see if the projectile is still in the world.  It will check things like
	* the KillZ, outside world bounds, etc. and handle the situation. */
	virtual bool CheckStillInWorld();

	/** Is the root component controlled by this component still? */
	UFUNCTION(BlueprintPure, Category = "Game|Components|RealisticProjectile")
	bool HasStoppedSimulation() { return UpdatedComponent == NULL; }

	//Begin UActorComponent Interface
	/**
	 * Calculate where the new position of the UpdatedComponent will be after DeltaTime and move it there. If there are hits
	 * on the way, do several intermediate calculations and moves until trajectory for DeltaTime is completed.
	 */
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;
	//End UActorComponent Interface

	//Begin UMovementComponent Interface.
	// TODO - gravity in any direction
	virtual float GetGravityZ() const override { return Settings.Gravity.Z; } 
	virtual void InitializeComponent() override;
	//End UMovementComponent Interface

protected:
	/** Data about where the projectile is at a given time, also the things that need to be updated every tick. Good to use as a return value. */
	struct SubTickState {
		FVector Location;
		FRotator Rotation;
		FVector Velocity;
		FTrajectoryInitialConditions Trajectory;
		float TickRemainingTime;

		SubTickState(){}

		SubTickState(const FVector& WorldLocation, const FRotator& WorldRotation, const FVector& WorldVelocity, const FTrajectoryInitialConditions& InitialConditions, float TickTimeRemaining)
		{
			Location = WorldLocation;
			Rotation = WorldRotation;
			Velocity = WorldVelocity;
			Trajectory = InitialConditions;
			TickRemainingTime = TickTimeRemaining;
		}

		/** Convenience function which stops you from forgetting to change something by the end of a tick. */
		void SetFromState(URealisticProjectileComponent* ComponentToUpdate, float& ChangeTickRemainingTime)
		{
			if (IsValid(ComponentToUpdate))
			{
				ComponentToUpdate->Velocity = Velocity;
				if (ComponentToUpdate->InitialConditions.t0 != Trajectory.t0) { ComponentToUpdate->InitialConditionHistory.Emplace(Trajectory); }
				ComponentToUpdate->InitialConditions = Trajectory;
				ComponentToUpdate->UpdateComponentVelocity();
				ComponentToUpdate->ActorMove(Location, Rotation);
			}
			//else {UE_LOG(ProjectilePhysics, Error, TEXT("Passed null to SubTickState::SetFromState")) }
			ChangeTickRemainingTime = TickRemainingTime;
		}
	};

	/** k = 0.5g / ||Vterminal|| as per Carpentier paper where g is gravity but positive. This is updated every tick
	 * so changes in world gravity and setting the terminal velocity effects this object. */
	float k;

	/** Vinf = Terminal Velocity + Wind Velocity in Carpentier paper.*/
	FVector Vinf = FVector(0.f, 0.f, -Settings.TerminalVelocity);
	
	/** Utility Function to move actor */
	virtual void ActorMove(const FVector& NewLocation, const FRotator& NewRotation);

	/* After exiting an object, call this to get a slightly different direction. Result is guaranteed to not be pointing
	* back in to the face defined by ImpactNormal */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ProjectileMovement")
	virtual FVector AdjustDirection(FVector InVelocity, FVector ImpactNormal);

	/* Apply impulse after hit*/
	UFUNCTION(BlueprintCallable, Category = "Game|Components|ProjectileMovement")
	virtual void ApplyImpulse(FHitResult& Hit, FVector ImpactVelocity) const;

	/* Get the other component and other actor and broadcast their OnHit or OnOverlap events.
	 * @param  Hit				The hit we just got */
	void BroadcastHitOrOverlap(FHitResult Hit);

	/**
	* Get the time taken to move from launch location to HitLocation.
	* Used when a hit is encountered during a tick. Needed to ensure the InitialConditions are
	* deterministic on different machines with different tick rates. See Carpentier paper for explanation
	* of equation. The function will subtract the current wind offset from the given parameter.
	* @param InitialConditions	Initial location, velocity, etc
	* @param HitLocation		Calculate time of flight from launch to this location
	* @param CurrentVelocity	The current velocity can decide whether the trajectory is before or after apex
	* @returns The time in seconds to move from launch (in InitialConditions) to HitLocation
	*/
	float ComputeTimeOfFlight(const FTrajectoryInitialConditions& InitialConditions, const FVector& HitLocation, const FVector& CurrentVelocity);

	/**
	* Get the point of exit for a specific component. Is called recursively until the hit closest to OrigLocation is reached
	* @param  OutHit				The first exit hit encountered will be stored here - if any.
	* @param  OrigLocation			The point to backwards sweep to. Usually the entry hit location
	* @param  NewLocation			The location of the projectile at the end of this tick
	* @param  World					The world object
	* @param  Component				Only find hits against this
	* @param  IgnoredComponents		A QueryParams object with components to ignore. Will be added to in this function.
	* @param  RecursionDepth		Function doesn't infinitely recurse in testing, but it doesn't hurt to prevent it.
	*								Currently capped at 64 calls, so exit hit returned may be early if for example a bullet
	*								travels through a single mesh house with 64 rooms in one tick.
	* @require						IgnoredComponents does not contain the searched for component
	* @ensure						For convenience, IgnoredComponents should have 0 ignored components after ultimately returning
	* @return true if a hit was found. The hit will be stored in OutHit
	*/
	virtual bool GetExitHit(
		FHitResult &OutHit,
		const FVector &OrigLocation,
		const FVector &NewLocation,
		const FRotator &Rotation,
		const UWorld* World,
		const UPrimitiveComponent* Component,
		FComponentQueryParams &IgnoredComponents,
		uint8 RecursionDepth = 0);
	
	/** When we are in an object - update the deceleration experienced due to that object's toughness */
	virtual float GetNewPenetrationDeceleration(FHitResult Hit);

	/* Get the other component and other actor and broadcast their EndOverlap event */
	void NotifyHitObjectOfEndOverlap(UPrimitiveComponent* OtherComponent);

private:
	/** The initial trajectory information - t0, v0, Location at t0 etc. */
	FTrajectoryInitialConditions InitialConditions;
	/** The table of material properties. This is assigned by path in the initializer */
	const UDataTable* MaterialPropertiesTable;
	/** Variables which are used each tick - save overhead by allocating memory here */
	TArray<struct FHitResult> MoveEntryHits;
	/** Must keep track of these to call endoverlap */
	TArray<UPrimitiveComponent*> OverlappedComponents;
	/** A buffer for hit results so functions that need a TArray don't have to allocate and free memory every tick */
	TArray<FHitResult> TempHitResults;
	/** When sliding with friction: last TimeSinceLaunch calculated */
	float FrLastLocationComputeTime = -1.f;
	/** When sliding with friction: last trajectory t0 calculated */
	float FrLastLocationComputeTrajectory = -1.f;
	/** When sliding with friction: last TimeSinceLaunch calculated */
	float FrLastRotationComputeTime = -1.f;
	/** When sliding with friction: last trajectory t0 calculated */
	float FrLastRotationComputeTrajectory = -1.f;
	/** When sliding with friction: last TimeSinceLaunch calculated */
	float FrLastVelocityComputeTime = -1.f;
	/** When sliding with friction: last trajectory t0 calculated */
	float FrLastVelocityComputeTrajectory = -1.f;
	/** When sliding with friction: size of gravity acceleration */
	float FrGravityComponentSizeSquared;
	
	/** Get acceleration from friction in cm/s/s */
	static float AccelerationDueToFriction(float Friction, const FVector& Gravity = FVector(0.f, 0.f, -980.f)) { return -Friction * Gravity.Size(); }

	/**
	* See if the projectile went off a ledge. If so, FinalLocation and TickRemainingTime will be adjusted, a new Initial Conditions is created.
	* @param	OutState
	* @param	PotentialState			Where the projectile would get to this subtick (if it is still on the floor)
	* @param	World					The world
	* @param	GameTime				The current game time
	* @param	TimeSinceLaunch			That.
	* @ensure	FinalLocation will contain the location for next tick, TickRemainingTime will contain just that, InitialConditions member
	*			has been changed.
	*/
	bool CheckStillSliding(struct SubTickState& OutState, const struct SubTickState& PotentialState, const UWorld* World, float GameTime, float TimeSinceLaunch);
	
#if WITH_EDITOR
	/* Get the right color and draw the debug line */
	void DoDrawDebugLine(FVector StartLocation, FVector NewLocation, float Velocity);
#endif

	/**
	* Sweep the updated component geometry, returning all hitresults in the parameter.
	* Does not move the updated component.
	* @param StartLocation		Beginning of sweep
	* @param EndLocation		End of sweep
	* @param Rot				The rotation of the component through the whole sweep
	* @param OutHits			Return hitresults from sweep here
	* @param World				The world object
	* @param KeepOverlaps		Include the hits that were only overlapping, not blocking in results
	* @param GetPhysicalMaterial Populate the physical material of the results
	* @returns true if a hit was found
	*/
	virtual bool DoSweep(FVector StartLocation,
		FVector EndLocation,
		FRotator Rot,
		TArray<struct FHitResult> &OutHits,
		const UWorld* World,
		bool KeepOverlaps = false,
		bool GetPhysicalMaterial = true);

	/**
	* Identify a projectile from the given actors with seed 'RandSeed' and if more than one, the closest to TimeSinceShot.
	* @param	World				The World obj
	* @param	ProjectileActors	Array of AActor* with valid RealisticProjectileComponents
	* @param	RandSeed			The seed to search for
	* @param	TimeSinceShot		The time since bullet was shot
	*/
	static AActor* FindProjectileByRandSeedAndTimeShot(const UWorld* World, const TArray<AActor*>& ProjectileActors, int32 RandSeed, float TimeSinceShot);

	/**
	 * Get a unit vector pointing in the positive x,y, or z direction, depending on which component of Vector is largest.
	 * @param	Vector			A vector to find the largest component of.
	 * @return A unit vector pointing in the x, y, or z direction.
	 */
	static FVector GetLargestDimension(const FVector& Vector);
	
	/**
	* Continue processing a tick after the new location is found, and we are in air. Any entry hits are in MoveEntryHits.
	* @param  OutState				The state of the projectile after checking for entry hits
	* @param  InState				The state of the projectile if it travels without hits this tick
	* @param  TickStartLocation		The location before moving this subtick
	* @param  TimeSinceLaunch		The gametime from the start of this subtick to predicted end of this tick.
	* @param  GameTime				The current game time from World.
	* @param  World					A valid UWorld object
	* @param  PredictOnly			If true, this projectile is imaginary and should not trigger hit or overlap events
	* @require MoveEntryHits is filled with any BLOCKING, non-starting hits from the start of the tick to FinalLocation.
	* @ensure	All members of OutState are correct for the end of this subtick
	*/
	void HandleAirTick(struct SubTickState& OutState, const struct SubTickState& PotentialState, const FVector& TickStartLocation, float TimeSinceLaunch, float GameTime, const UWorld* World, bool PredictOnly);

	/**
	* Continue processing a tick after the new location is found and we are inside an object. Any entry hits are in MoveEntryHits. At the end of this function a bunch
	* of state could have changed: InitialConditions, Velocity, ObjectsPenetrated etc.
	* @param  OutState				The state of the projectile after checking for entry hits
	* @param  InState				The state of the projectile if it travels without hits this tick
	* @param  TickStartLocation		The location before moving this subtick
	* @param  TimeSinceLaunch		The gametime from the start of this subtick to predicted end of this tick.
	* @param  GameTime				The current game time from World.
	* @param  World					A valid UWorld object
	* @param  PredictOnly			If true, this projectile is imaginary and should not trigger hit or overlap events
	* @require MoveEntryHits is filled with any BLOCKING, non-starting hits from the start of the tick to FinalLocation.
	* @ensure	All members of OutState are correct for the end of this subtick
	*/
	void HandleInObjectTick(struct SubTickState& OutState, const struct SubTickState& InState, float GameTime, const UWorld* World, bool PredictOnly);
	
	/**
	* The exact formula for computing deceleration, in one place. For general use call GetNewPenetrationDeceleration(FHitResult) which calls this.
	* @param  Toughness				The toughness value of the material hit
	* @param  DecelerationExponent	The current deceleration exponent
	* @param  DecelerationCoefficient	The current deceleration coefficient
	* @return A negative value representing acceleration in the direction of movement.
	*/
	static float InternalGetPenetrationDeceleration(float Toughness, float DecelerationExponent, float DecelerationCoefficient);

	/** Helper for GetPredictedTrajectoryForSettings and other prediction functions. */
	static void InternalGetPredictedTrajectoryForSettings(UObject* WorldContextObject, TArray<FVector>& WorldLocations, FVector& LastVelocity, FVector StartLocation, FRotator StartRotation, 
		TSubclassOf<AActor> Properties, const FRealisticProjectileBehavior* Settings, int32 RandSeed = 0, float TimeToSimulate = 3.f, float StepTime = 0.05f, int32 MaxPoints = 200,
		bool bOverrideGravity = false, FVector GravityAccel = FVector(0.f, 0.f, -980.f), FVector Scale = FVector(1,1,1));

	/**
	* @require All arguments are valid and not null
	*/
	static void InternalPredictTrajectory(UWorld* World, TArray<FVector>& WorldLocations, TArray<FRotator>& Rotations, TArray<FVector>& Velocities, FVector& LastVelocity,
		AActor* Instance, URealisticProjectileComponent* SpawnedComponent, const FVector& StartLocation, const FRotator& StartRotation, float TimeToSimulate, 
		float StepTime, int32 MaxPoints, bool LocationsOnly = true);

	static bool InternalShouldBounce(const URealisticProjectileComponent* Proj, const FHitResult& Hit, float Toughness, const FVector& Velocity, float Roll = -1.f);

	/**
	* Subtick function for both actual and predicted trajectories. Tries not to modify the state of self/UpdatedComponent at all (but still does when sliding with friction).
	* TODO make self const, remove predictonly
	* @param	self			A RPComponent, could be for a prediction only
	* @param	OutState		The state self should be set to at the end of the subtick
	* @param	InState			The state of self before the subtick
	* @param	World			The World object
	* @param	GameTime		The game time in seconds, affected by pause/dilate
	* @param	PredictOnly		If true, does not broadcast hit or overlap events and such
	*/
	static void InternalSubTick(URealisticProjectileComponent* self, struct SubTickState& OutState, const struct SubTickState& InState, 
		const UWorld* World, float GameTime, bool PredictOnly = false);

	/** 
	* Return true if the projectile has come to a stop.
	* @param  Self			A RPComponent with correct velocity and initialconditions for the calculation
	* @param  PredictOnly	If true, do not broadcast OnProjectileStop
	*/
	static bool HasStoppedMoving(URealisticProjectileComponent* Self, bool PredictOnly = false);

	/** Spawn an actor and get its RPComponent. Change the settings of RPComponent to the parameter and reinitialize it.
	* @param	OutActor		Returned spawned actor
	* @param	OutComponent	Return spawned component
	* @param	OutWorld		Return UWorld if GEngine works
	* @param	StartLocation	Spawn the actor here
	* @param	StartRotation	Spawn the actor this way
	* @param	ActorClass		An actor class with a realistic projectile component and a primitive rootcomponent
	* @param	RandSeed		The RNG for the projectile
	* @param	GravityAccel	Convenience param for force of gravity in cm/s/s
	* @param	Scale			Convenience param for spawned scale
	* @param	Settings		Optionally set the RPComponent settings to this after spawning
	*/
	static bool SetupPredictedTrajectory(UObject* WorldContextObject, AActor*& OutActor, URealisticProjectileComponent*& OutComponent, UWorld*& OutWorld,
		 const FVector& StartLocation, const FRotator& StartRotation, TSubclassOf<AActor> Properties, int32 RandSeed, 
		 bool bOverrideGravity, const FVector& GravityAccel, const FVector& Scale, 
		 const FRealisticProjectileBehavior* Settings = nullptr);

	/**
	* Get the initial conditions for sliding next tick.
	* @param	OutState				The trajectory and other tick information will be here
	* @param	InState					The state of the projectile AT hit time.
	* @param	GameTime				The current game time
	* @param	Hit						The hit result of the surface to start sliding
	* @param	FlightTime				Time since last trajectory start
	* @require	The surface hit is not a ceiling (ImpactNormal.Z faces opposite direction to gravity)
	* @ensure	All members of OutState represent the state at the end of this subtick.
	*/
	virtual void StartSliding(struct SubTickState& OutState, const struct SubTickState& InState, float GameTime, const FHitResult& Hit, float FlightTime, bool PredictOnly);

	/**
	 * Change OutState to an in air trajectory with a new post-sliding transform and velocity.
	 * @param	InState					State of projectile after attempted move that was found to be off ledge
	 * @param	StartTickLocation		Where the projectile is at the start of the tick
	 */
	virtual void StopSliding(struct SubTickState& OutState, const struct SubTickState& InState, const FVector& StartTickLocation, const UWorld* World, float GameTime);
	
	/* If conditions are met, turn collision off and attach to hit component, otherwise start simulating regular physics*/
	virtual void TryEmbed();
	
#if WITH_EDITOR
	/** Controls which UPROPERTYs are disabled in editor depending on conditions. */
	virtual bool CanEditChange(const UProperty* InProperty) const override;
#endif
};