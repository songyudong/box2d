// SPDX-FileCopyrightText: 2024
// SPDX-License-Identifier: MIT

// Skiing scene: A skier slides down a curved snow slope with keyboard control,
// gravity-driven airborne motion, and balance stabilization.

#include "draw.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <array>
#include <algorithm>
#include <vector>

class Skiing : public Sample
{
public:
	struct SkierRig
	{
		b2BodyId torsoId = b2_nullBodyId;
		b2BodyId headId = b2_nullBodyId;
		b2BodyId leftArmId = b2_nullBodyId;
		b2BodyId rightArmId = b2_nullBodyId;
		b2BodyId leftLegId = b2_nullBodyId;
		b2BodyId rightLegId = b2_nullBodyId;
		b2BodyId skiId = b2_nullBodyId;

		b2JointId headJointId = b2_nullJointId;
		b2JointId leftArmJointId = b2_nullJointId;
		b2JointId rightArmJointId = b2_nullJointId;
		b2JointId leftLegJointId = b2_nullJointId;
		b2JointId rightLegJointId = b2_nullJointId;
		b2JointId skiWeldJointId = b2_nullJointId;

		bool prevJumpPressed = false;
		int jumpBufferSteps = 0;
		int coyoteSteps = 0;
		int reverseSteps = 0;
		bool recoveryJumpArmed = false;
		int stuckSteps = 0;
		float cruiseSpeed = 0.0f;
		float boostTime = 0.0f;
		float parachuteTime = 0.0f;
	};

	struct RocketPickup
	{
		b2Vec2 position = { 0.0f, 0.0f };
		bool collected = false;
	};

	struct ParachutePickup
	{
		b2Vec2 position = { 0.0f, 0.0f };
		bool collected = false;
	};

	struct CloudInstance
	{
		b2Vec2 position = { 0.0f, 0.0f };
		float scale = 1.0f;
	};

	struct LeafInstance
	{
		b2Vec2 position = { 0.0f, 0.0f };
		float angle = 0.0f;
		float scale = 1.0f;
		b2HexColor color = b2_colorGold;
	};

	struct TreeInstance
	{
		b2Vec2 position = { 0.0f, 0.0f };
		float trunkHeight = 0.0f;
		float trunkHalfWidth = 0.0f;
		float foliageWidth = 0.0f;
		float foliageHeight = 0.0f;
	};

	struct FlowerInstance
	{
		b2Vec2 position = { 0.0f, 0.0f };
		float stemHeight = 0.0f;
		float petalRadius = 0.0f;
		b2HexColor petalColor = b2_colorHotPink;
	};

	explicit Skiing( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { -20.0f, 10.0f };
			m_context->camera.zoom = 25.0f * 1.2f;
		}

		// Default gravity is (0, -10) in Box2D samples; we use (0, -9.8)
		b2World_SetGravity( m_worldId, { 0.0f, -9.8f } );

		CreateGround();
		CreateRocks();
		CreateRocketPickups();
		CreateParachutePickups();
		CreateFinishFlag();
		CreateCastle();
		CreateScenery();
		CreateSkier( m_player, { -45.0f, 22.0f }, false );

		m_forceStrength = 50.0f;
		m_maxSpeed = 15.0f;
		m_jumpImpulse = 5.0f;
		m_jumpBufferDuration = 30;
		m_coyoteDuration = 8;
		m_boostDuration = 10.0f;
		m_boostSpeedMultiplier = 2.0f;
		m_parachuteDuration = 10.0f;
		m_aiCruiseSpeed = 15.0f;
		m_aiJumpLookAhead = 7.0f;
		m_aiJumpHeightTolerance = 0.8f;
		m_aiReverseDuration = 28;
		m_aiStuckSpeedThreshold = 1.5f;
		m_aiRespawnTiltCos = 0.55f;
		m_aiRespawnStuckSteps = 90;
		m_aiRocketSeekRange = 18.0f;
		m_aiRocketSeekHeightTolerance = 6.0f;
		m_aiRocketJumpHeight = 1.2f;
		m_aiMaxCruiseSpeed = 17.0f;
		m_balanceTorque = 10.0f;
		m_balanceAngleLimit = 15.0f * B2_PI / 180.0f;
		m_airBalanceStrength = 20.0f;
		m_airDampingStrength = 5.0f;
		m_followCamera = true;
		m_cameraTargetIndex = -1;

		for ( int i = 0; i < m_aiCount; ++i )
		{
			float x = -42.0f + 3.0f * i;
			float y = 24.0f + 1.3f * float( i % 6 );
			CreateSkier( m_aiSkiers[i], { x, y }, true );
			float t = m_aiCount > 1 ? float( i ) / float( m_aiCount - 1 ) : 0.0f;
			m_aiSkiers[i].cruiseSpeed = 0.90f * m_aiCruiseSpeed + t * ( m_aiMaxCruiseSpeed - 0.90f * m_aiCruiseSpeed );
		}
	}

	void CreateScenery()
	{
		m_clouds.clear();
		m_leaves.clear();
		m_trees.clear();
		m_flowers.clear();

		float cloudEndX = GetTrackEndX() - 30.0f;
		for ( int i = 0; i < 72; ++i )
		{
			float t = i / 71.0f;
			float anchorX = -25.0f + t * ( cloudEndX + 25.0f ) + 12.0f * sinf( 0.71f * i );
			float anchorY = GetTerrainHeight( anchorX ) + 20.0f + 7.0f * ( 0.5f + 0.5f * sinf( 1.19f * i ) );
			int clusterCount = 2 + ( i % 3 );
			for ( int j = 0; j < clusterCount; ++j )
			{
				float x = anchorX + ( j - 0.5f * float( clusterCount - 1 ) ) * 2.6f;
				float y = anchorY + 0.4f * sinf( 0.93f * ( i + j ) );
				float scale = ( 0.78f + 0.22f * j ) * ( 0.9f + 0.35f * ( 0.5f + 0.5f * sinf( 0.83f * i + 0.41f * j ) ) );
				m_clouds.push_back( { { x, y }, scale } );
			}
		}

		float leafEndX = GetTrackEndX() - 25.0f;
		for ( int i = 0; i < 192; ++i )
		{
			float t = i / 191.0f;
			float x = -35.0f + t * ( leafEndX + 35.0f ) + 6.0f * sinf( 0.83f * i );
			float groundY = GetTerrainHeight( x );
			float angle = 0.35f * sinf( 1.57f * i );
			float leafScale = 0.18f + 0.06f * ( 0.5f + 0.5f * sinf( 1.11f * i ) );
			b2HexColor leafColor = ( i % 3 == 0 ) ? b2_colorGold : ( i % 3 == 1 ) ? b2_colorOrange : b2_colorFireBrick;
			m_leaves.push_back( { { x, groundY + 0.03f }, angle, leafScale, leafColor } );
		}

		float treeEndX = GetTrackEndX() - 80.0f;
		for ( int i = 0; i < 88; ++i )
		{
			float t = i / 87.0f;
			float x = -40.0f + t * ( treeEndX + 40.0f ) + 16.0f * sinf( 0.91f * i );
			float groundY = GetTerrainHeight( x );
			float trunkHeight = 1.8f + 0.4f * ( 0.5f + 0.5f * sinf( 1.71f * i ) );
			float trunkHalfWidth = 0.22f + 0.04f * ( 0.5f + 0.5f * sinf( 0.63f * i ) );
			float foliageWidth = 1.8f + 0.3f * ( 0.5f + 0.5f * sinf( 1.13f * i ) );
			float foliageHeight = 2.4f + 0.5f * ( 0.5f + 0.5f * sinf( 1.97f * i ) );
			m_trees.push_back( { { x, groundY + 0.02f }, trunkHeight, trunkHalfWidth, foliageWidth, foliageHeight } );
		}

		float flowerEndX = GetTrackEndX() - 20.0f;
		for ( int i = 0; i < 144; ++i )
		{
			float t = i / 143.0f;
			float x = -30.0f + t * ( flowerEndX + 30.0f ) + 8.0f * sinf( 1.23f * i );
			float groundY = GetTerrainHeight( x );
			float stemHeight = 0.45f + 0.12f * ( 0.5f + 0.5f * sinf( 0.77f * i ) );
			float petalRadius = 0.12f + 0.03f * ( 0.5f + 0.5f * sinf( 1.61f * i ) );
			b2HexColor petalColor = ( i % 3 == 0 ) ? b2_colorHotPink : ( i % 3 == 1 ) ? b2_colorGold : b2_colorLavender;
			m_flowers.push_back( { { x, groundY }, stemHeight, petalRadius, petalColor } );
		}
	}

	bool IsXVisible( float x, float margin ) const
	{
		float ratio = m_context->camera.width / m_context->camera.height;
		float halfWidth = m_context->camera.zoom * ratio;
		return x >= m_context->camera.center.x - halfWidth - margin && x <= m_context->camera.center.x + halfWidth + margin;
	}

	void CreateParachutePickups()
	{
		m_parachutePickups.clear();
		float endX = GetTrackEndX() - 30.0f;
		for ( int i = 0; i < 44; ++i )
		{
			float t = i / 43.0f;
			float cloudX = -25.0f + t * ( endX + 25.0f ) + 12.0f * sinf( 0.71f * i );
			float cloudY = GetTerrainHeight( cloudX ) + 20.0f + 7.0f * ( 0.5f + 0.5f * sinf( 1.19f * i ) );

			int clusterCount = 2 + ( i % 3 );
			for ( int j = 0; j < clusterCount; ++j )
			{
				float xOffset = ( j - 0.5f * float( clusterCount - 1 ) ) * 1.0f;
				float yOffset = -0.9f - 0.35f * float( j % 2 ) + 0.15f * sinf( 1.27f * ( i + j ) );
				b2Vec2 candidate = { cloudX + xOffset, cloudY + yOffset };

				bool overlapsRocket = false;
				for ( const RocketPickup& rocket : m_rocketPickups )
				{
					if ( b2Distance( rocket.position, candidate ) < 1.8f )
					{
						overlapsRocket = true;
						break;
					}
				}

				if ( overlapsRocket == false )
				{
					m_parachutePickups.push_back( { candidate, false } );
				}
			}
		}
	}

	void DrawClouds()
	{
		for ( const CloudInstance& cloud : m_clouds )
		{
			if ( IsXVisible( cloud.position.x, 4.0f ) == false )
			{
				continue;
			}

			float x = cloud.position.x;
			float y = cloud.position.y;
			float scale = cloud.scale;
			DrawSolidCircle( m_draw, { { x - 1.10f * scale, y }, b2MakeRot( 0.0f ) }, 0.55f * scale, b2_colorWhite );
			DrawSolidCircle( m_draw, { { x, y + 0.20f * scale }, b2MakeRot( 0.0f ) }, 0.72f * scale, b2_colorWhite );
			DrawSolidCircle( m_draw, { { x + 1.24f * scale, y + 0.05f * scale }, b2MakeRot( 0.0f ) }, 0.50f * scale, b2_colorWhite );
			DrawSolidCapsule( m_draw, { x - 1.70f * scale, y - 0.20f * scale }, { x + 1.84f * scale, y - 0.18f * scale },
				0.28f * scale, b2_colorWhite );
		}
	}

	void DrawParachutePickups()
	{
		for ( const ParachutePickup& parachute : m_parachutePickups )
		{
			if ( parachute.collected )
			{
				continue;
			}

			b2Transform canopyTransform = { { parachute.position.x, parachute.position.y + 0.2f }, b2MakeRot( 0.0f ) };
			b2Vec2 canopy[5] = {
				{ -0.75f, 0.0f },
				{ -0.45f, 0.42f },
				{ 0.0f, 0.58f },
				{ 0.45f, 0.42f },
				{ 0.75f, 0.0f },
			};
			DrawSolidPolygon( m_draw, canopyTransform, canopy, 5, 0.0f, b2_colorHotPink );
			DrawLine( m_draw, { parachute.position.x - 0.48f, parachute.position.y + 0.20f }, { parachute.position.x - 0.15f, parachute.position.y - 0.55f }, b2_colorWhite );
			DrawLine( m_draw, { parachute.position.x + 0.48f, parachute.position.y + 0.20f }, { parachute.position.x + 0.15f, parachute.position.y - 0.55f }, b2_colorWhite );
			DrawLine( m_draw, { parachute.position.x, parachute.position.y + 0.30f }, { parachute.position.x, parachute.position.y - 0.55f }, b2_colorWhite );
			b2Vec2 harness[4] = {
				{ -0.12f, -0.65f },
				{ 0.12f, -0.65f },
				{ 0.12f, -0.45f },
				{ -0.12f, -0.45f },
			};
			DrawSolidPolygon( m_draw, { parachute.position, b2MakeRot( 0.0f ) }, harness, 4, 0.0f, b2_colorLightSteelBlue );
		}
	}

	void DrawParachuteEffect( const SkierRig& skier )
	{
		if ( skier.parachuteTime <= 0.0f )
		{
			return;
		}

		b2Vec2 pos = b2Body_GetPosition( skier.torsoId );
		b2Transform canopyTransform = { { pos.x, pos.y + 1.7f }, b2MakeRot( 0.0f ) };
		b2Vec2 canopy[5] = {
			{ -0.95f, 0.0f },
			{ -0.58f, 0.52f },
			{ 0.0f, 0.72f },
			{ 0.58f, 0.52f },
			{ 0.95f, 0.0f },
		};
		DrawSolidPolygon( m_draw, canopyTransform, canopy, 5, 0.0f, b2_colorDeepPink );
		DrawLine( m_draw, { pos.x - 0.62f, pos.y + 1.70f }, { pos.x - 0.18f, pos.y + 0.38f }, b2_colorWhite );
		DrawLine( m_draw, { pos.x + 0.62f, pos.y + 1.70f }, { pos.x + 0.18f, pos.y + 0.38f }, b2_colorWhite );
		DrawLine( m_draw, { pos.x, pos.y + 1.82f }, { pos.x, pos.y + 0.34f }, b2_colorWhite );
	}

	void CreateRocketPickups()
	{
		m_rocketPickups.clear();
		float endX = GetTrackEndX() - 60.0f;
		for ( int i = 0; i < 48; ++i )
		{
			float t = i / 47.0f;
			float x = -10.0f + t * ( endX + 10.0f ) + 14.0f * sinf( 0.93f * i );
			float groundY = GetTerrainHeight( x );
			bool airbornePickup = ( i % 4 ) != 0;
			float y = airbornePickup ? groundY + 9.0f + 4.0f * ( 0.5f + 0.5f * sinf( 1.17f * i ) ) : groundY + 1.0f;

			int clusterCount = 2 + ( i % 3 );
			for ( int j = 0; j < clusterCount; ++j )
			{
				float xOffset = ( j - 0.5f * float( clusterCount - 1 ) ) * ( airbornePickup ? 1.3f : 0.9f );
				float yOffset = airbornePickup ? 0.8f * sinf( 1.41f * ( i + j ) ) : 0.35f * sinf( 1.13f * ( i + j ) );
				m_rocketPickups.push_back( { { x + xOffset, y + yOffset }, false } );
			}
		}
	}

	void UpdateParachutePickups()
	{
		for ( ParachutePickup& parachute : m_parachutePickups )
		{
			if ( parachute.collected )
			{
				continue;
			}

			TryCollectParachute( parachute, m_player );
			for ( SkierRig& skier : m_aiSkiers )
			{
				TryCollectParachute( parachute, skier );
			}
		}
	}

	float GetTerrainHeight( float x ) const
	{
		return 18.0f - 0.02f * ( x + 55.0f )
			   + 12.0f * sinf( 0.008f * x )
			   + 6.0f * sinf( 0.025f * x )
			   + 3.0f * sinf( 0.08f * x )
			   + 1.0f * sinf( 0.25f * x )
			   + 0.3f * sinf( 0.6f * x );
	}

	float GetTrackEndX() const
	{
		return -55.0f + 1.5f * ( m_trackSegCount + 3 );
	}

	// ©¤©¤©¤ Curved snow slope using b2ChainDef (open chain) ©¤©¤©¤
	void CreateGround()
	{
		// Generate a smooth undulating ski slope.
		// The terrain descends overall with hills and valleys.
		// Chain shapes are one-sided: the collision normal points to the right
		// of the segment direction. Points go right-to-left so that the normal
		// points upward, allowing collision from above.
		// We add ghost points at both ends for smooth collision on open chains.
		const int segCount = m_trackSegCount;
		b2Vec2 points[segCount + 4]; // +4 for ghost vertices (2 at each end)

		float x = -55.0f;
		float dx = 1.5f;

		// Generate terrain with a general downward slope and tall hills
		for ( int i = 0; i < segCount + 4; ++i )
		{
			float t = x;
			// Overall descent with multi-scale sinusoidal bumps to create mountains and hills
			float y = 18.0f - 0.02f * ( t + 55.0f )  // gentle overall descent (reduced for longer terrain)
					  + 12.0f * sinf( 0.008f * t )    // huge mountain-scale undulations
					  + 6.0f * sinf( 0.025f * t )     // large rolling hills
					  + 3.0f * sinf( 0.08f * t )      // medium hills
					  + 1.0f * sinf( 0.25f * t )      // small bumps
					  + 0.3f * sinf( 0.6f * t );      // fine undulations
			points[i] = { x, y };
			x += dx;
		}

		// Reverse points so the chain goes right-to-left, making the
		// one-sided collision normal point upward.
		for ( int i = 0; i < ( segCount + 4 ) / 2; ++i )
		{
			b2Vec2 tmp = points[i];
			points[i] = points[segCount + 3 - i];
			points[segCount + 3 - i] = tmp;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &bodyDef );

		// Surface material: low friction to simulate snow
		b2SurfaceMaterial snowMaterial = b2DefaultSurfaceMaterial();
		snowMaterial.friction = 0.05f;         // dynamic friction (snow)
		snowMaterial.restitution = 0.0f;
		snowMaterial.customColor = b2_colorWhite;

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.count = segCount + 4;
		chainDef.materials = &snowMaterial;
		chainDef.materialCount = 1;
		chainDef.isLoop = false; // open chain; first/last edges are ghost edges

		m_chainId = b2CreateChain( m_groundId, &chainDef );

		b2ShapeDef extensionShapeDef = b2DefaultShapeDef();
		extensionShapeDef.material.friction = 0.05f;
		extensionShapeDef.material.restitution = 0.0f;
		extensionShapeDef.material.customColor = b2_colorWhite;

		float trackEndX = GetTrackEndX();
		float extensionHalfWidth = 12.0f;
		float extensionCenterX = trackEndX + extensionHalfWidth;
		float extensionGroundY = GetTerrainHeight( trackEndX );
		b2Polygon extension = b2MakeOffsetBox( extensionHalfWidth, 1.0f, { extensionCenterX, extensionGroundY - 1.0f }, b2MakeRot( 0.0f ) );
		b2CreatePolygonShape( m_groundId, &extensionShapeDef, &extension );
	}

	void CreateFinishFlag()
	{
		float x = GetTrackEndX() - 6.0f;
		float groundY = GetTerrainHeight( x );
		float poleHalfHeight = 18.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { x, groundY + poleHalfHeight };
		bodyDef.name = "finish_flag";
		m_finishId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef poleShapeDef = b2DefaultShapeDef();
		poleShapeDef.material.friction = 0.6f;
		poleShapeDef.material.customColor = b2_colorLightSlateGray;

		b2Polygon pole = b2MakeOffsetBox( 0.12f, poleHalfHeight, { 0.0f, 0.0f }, b2MakeRot( 0.0f ) );
		b2CreatePolygonShape( m_finishId, &poleShapeDef, &pole );

		b2Polygon stopper = b2MakeOffsetBox( 0.9f, 0.45f, { -0.8f, -poleHalfHeight + 0.5f }, b2MakeRot( 0.0f ) );
		b2CreatePolygonShape( m_finishId, &poleShapeDef, &stopper );

		b2ShapeDef flagShapeDef = b2DefaultShapeDef();
		flagShapeDef.material.friction = 0.2f;
		flagShapeDef.material.customColor = b2_colorCrimson;
		b2Polygon flag = b2MakeOffsetBox( 1.6f, 0.65f, { 1.7f, poleHalfHeight - 1.5f }, b2MakeRot( 0.0f ) );
		b2CreatePolygonShape( m_finishId, &flagShapeDef, &flag );
	}

	void CreateCastle()
	{
		float x = GetTrackEndX() + 10.0f;
		float groundY = GetTerrainHeight( GetTrackEndX() );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { x, groundY };
		bodyDef.name = "castle";
		m_castleId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef wallShapeDef = b2DefaultShapeDef();
		wallShapeDef.material.friction = 0.7f;
		wallShapeDef.material.restitution = 0.0f;
		wallShapeDef.material.customColor = b2_colorLightSlateGray;

		b2Polygon mainWall = b2MakeOffsetBox( 8.0f, 8.0f, { 0.0f, 8.0f }, b2MakeRot( 0.0f ) );
		b2CreatePolygonShape( m_castleId, &wallShapeDef, &mainWall );

		b2Polygon leftTower = b2MakeOffsetBox( 2.6f, 12.0f, { -6.8f, 12.0f }, b2MakeRot( 0.0f ) );
		b2CreatePolygonShape( m_castleId, &wallShapeDef, &leftTower );

		b2Polygon rightTower = b2MakeOffsetBox( 2.6f, 13.5f, { 6.8f, 13.5f }, b2MakeRot( 0.0f ) );
		b2CreatePolygonShape( m_castleId, &wallShapeDef, &rightTower );

		b2Polygon gateBlock = b2MakeOffsetBox( 3.0f, 5.5f, { 0.0f, 5.5f }, b2MakeRot( 0.0f ) );
		b2CreatePolygonShape( m_castleId, &wallShapeDef, &gateBlock );

		b2Polygon keep = b2MakeOffsetBox( 4.0f, 5.0f, { 0.0f, 18.5f }, b2MakeRot( 0.0f ) );
		b2CreatePolygonShape( m_castleId, &wallShapeDef, &keep );
	}

	void CreateRocks()
	{
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.6f;
		shapeDef.material.restitution = 0.0f;
		shapeDef.material.customColor = b2_colorGray;
		float endX = GetTrackEndX() - 90.0f;

		for ( int i = 1; i < 64; ++i )
		{
			float t = i / 63.0f;
			float x = -20.0f + t * ( endX + 20.0f ) + 20.0f * sinf( 1.37f * i );
			float radius = 1.20f + 0.80f * ( 0.5f + 0.5f * sinf( 2.11f * i ) );
			bool useTrapezoid = ( i % 2 ) == 0;
			float halfHeight = 0.65f * radius;
			float y = GetTerrainHeight( x ) + ( useTrapezoid ? halfHeight : radius ) + 0.04f;
			float slopeAngle = 0.0f;
			if ( useTrapezoid )
			{
				float sampleDx = 0.25f;
				float leftY = GetTerrainHeight( x - sampleDx );
				float rightY = GetTerrainHeight( x + sampleDx );
				slopeAngle = atan2f( rightY - leftY, 2.0f * sampleDx );
			}

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { x, y };
			bodyDef.rotation = b2MakeRot( slopeAngle );
			bodyDef.name = "rock";
			b2BodyId rockId = b2CreateBody( m_worldId, &bodyDef );

			if ( useTrapezoid )
			{
				float bottomHalfWidth = 0.95f * radius;
				float topHalfWidth = 0.55f * radius;
				b2Vec2 points[4] = {
					{ -bottomHalfWidth, -halfHeight },
					{ bottomHalfWidth, -halfHeight },
					{ topHalfWidth, halfHeight },
					{ -topHalfWidth, halfHeight },
				};
				b2Hull hull = b2ComputeHull( points, 4 );
				b2Polygon rock = b2MakePolygon( &hull, 0.0f );
				b2CreatePolygonShape( rockId, &shapeDef, &rock );
			}
			else
			{
				b2Circle rock = { { 0.0f, 0.0f }, radius };
				b2CreateCircleShape( rockId, &shapeDef, &rock );
			}
			m_rockIds.push_back( rockId );
		}
	}

	void DrawRocketPickups()
	{
		for ( const RocketPickup& rocket : m_rocketPickups )
		{
			if ( rocket.collected )
			{
				continue;
			}

			b2Transform transform = { rocket.position, b2MakeRot( 0.0f ) };
			b2Vec2 body[3] = {
				{ -0.18f, -0.45f },
				{ 0.18f, -0.45f },
				{ 0.0f, 0.30f },
			};
			b2Vec2 nose[3] = {
				{ -0.14f, 0.30f },
				{ 0.14f, 0.30f },
				{ 0.0f, 0.58f },
			};
			b2Vec2 finLeft[3] = {
				{ -0.18f, -0.30f },
				{ -0.35f, -0.52f },
				{ -0.05f, -0.42f },
			};
			b2Vec2 finRight[3] = {
				{ 0.18f, -0.30f },
				{ 0.35f, -0.52f },
				{ 0.05f, -0.42f },
			};
			DrawSolidPolygon( m_draw, transform, body, 3, 0.0f, b2_colorSilver );
			DrawSolidPolygon( m_draw, transform, nose, 3, 0.0f, b2_colorRed );
			DrawSolidPolygon( m_draw, transform, finLeft, 3, 0.0f, b2_colorOrangeRed );
			DrawSolidPolygon( m_draw, transform, finRight, 3, 0.0f, b2_colorOrangeRed );
		}
	}

	void DrawBoostEffect( const SkierRig& skier )
	{
		if ( skier.boostTime <= 0.0f )
		{
			return;
		}

		b2Vec2 pos = b2Body_GetPosition( skier.skiId );
		b2Vec2 vel = b2Body_GetLinearVelocity( skier.skiId );
		float dir = vel.x >= 0.0f ? -1.0f : 1.0f;
		b2Transform transform = { { pos.x + 0.9f * dir, pos.y - 0.05f }, b2MakeRot( 0.0f ) };
		b2Vec2 flame1[3] = {
			{ 0.0f, 0.0f },
			{ 0.35f * dir, 0.16f },
			{ 0.95f * dir, 0.0f },
		};
		b2Vec2 flame2[3] = {
			{ 0.0f, 0.0f },
			{ 0.28f * dir, -0.16f },
			{ 0.80f * dir, 0.0f },
		};
		DrawSolidPolygon( m_draw, transform, flame1, 3, 0.0f, b2_colorOrangeRed );
		DrawSolidPolygon( m_draw, transform, flame2, 3, 0.0f, b2_colorGold );
	}

	void UpdateRocketPickups()
	{
		for ( RocketPickup& rocket : m_rocketPickups )
		{
			if ( rocket.collected )
			{
				continue;
			}

			TryCollectRocket( rocket, m_player );
			for ( SkierRig& skier : m_aiSkiers )
			{
				TryCollectRocket( rocket, skier );
			}
		}
	}

	void TryCollectRocket( RocketPickup& rocket, SkierRig& skier )
	{
		if ( rocket.collected )
		{
			return;
		}

		b2Vec2 torsoPos = b2Body_GetPosition( skier.torsoId );
		b2Vec2 headPos = b2Body_GetPosition( skier.headId );
		b2Vec2 leftArmPos = b2Body_GetPosition( skier.leftArmId );
		b2Vec2 rightArmPos = b2Body_GetPosition( skier.rightArmId );
		b2Vec2 leftLegPos = b2Body_GetPosition( skier.leftLegId );
		b2Vec2 rightLegPos = b2Body_GetPosition( skier.rightLegId );
		b2Vec2 skiPos = b2Body_GetPosition( skier.skiId );
		if ( b2Distance( torsoPos, rocket.position ) < 1.08f || b2Distance( headPos, rocket.position ) < 1.08f ||
			 b2Distance( leftArmPos, rocket.position ) < 1.08f || b2Distance( rightArmPos, rocket.position ) < 1.08f ||
			 b2Distance( leftLegPos, rocket.position ) < 1.08f || b2Distance( rightLegPos, rocket.position ) < 1.08f ||
			 b2Distance( skiPos, rocket.position ) < 1.2f )
		{
			rocket.collected = true;
			skier.boostTime = m_boostDuration;
		}
	}

	void TryCollectParachute( ParachutePickup& parachute, SkierRig& skier )
	{
		if ( parachute.collected )
		{
			return;
		}

		b2Vec2 torsoPos = b2Body_GetPosition( skier.torsoId );
		b2Vec2 headPos = b2Body_GetPosition( skier.headId );
		b2Vec2 leftArmPos = b2Body_GetPosition( skier.leftArmId );
		b2Vec2 rightArmPos = b2Body_GetPosition( skier.rightArmId );
		b2Vec2 leftLegPos = b2Body_GetPosition( skier.leftLegId );
		b2Vec2 rightLegPos = b2Body_GetPosition( skier.rightLegId );
		b2Vec2 skiPos = b2Body_GetPosition( skier.skiId );
		if ( b2Distance( torsoPos, parachute.position ) < 1.2f || b2Distance( headPos, parachute.position ) < 1.2f ||
			 b2Distance( leftArmPos, parachute.position ) < 1.2f || b2Distance( rightArmPos, parachute.position ) < 1.2f ||
			 b2Distance( leftLegPos, parachute.position ) < 1.2f || b2Distance( rightLegPos, parachute.position ) < 1.2f ||
			 b2Distance( skiPos, parachute.position ) < 1.32f )
		{
			parachute.collected = true;
			skier.parachuteTime = m_parachuteDuration;
		}
	}

	void UpdateBoosts()
	{
		float dt = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 1.0f / 60.0f;
		m_player.boostTime = b2MaxFloat( 0.0f, m_player.boostTime - dt );
		m_player.parachuteTime = b2MaxFloat( 0.0f, m_player.parachuteTime - dt );
		for ( SkierRig& skier : m_aiSkiers )
		{
			skier.boostTime = b2MaxFloat( 0.0f, skier.boostTime - dt );
			skier.parachuteTime = b2MaxFloat( 0.0f, skier.parachuteTime - dt );
		}
	}

	void DrawFallenLeaves()
	{
		for ( const LeafInstance& leaf : m_leaves )
		{
			if ( IsXVisible( leaf.position.x, 1.0f ) == false )
			{
				continue;
			}

			b2Vec2 leafVertices[4] = {
				{ -0.9f * leaf.scale, 0.0f },
				{ 0.0f, 0.45f * leaf.scale },
				{ 0.9f * leaf.scale, 0.0f },
				{ 0.0f, -0.45f * leaf.scale },
			};

			b2Transform leafTransform = { leaf.position, b2MakeRot( leaf.angle ) };
			DrawSolidPolygon( m_draw, leafTransform, leafVertices, 4, 0.0f, leaf.color );
			DrawLine( m_draw,
				b2TransformPoint( leafTransform, { -0.7f * leaf.scale, 0.0f } ),
				b2TransformPoint( leafTransform, { 0.7f * leaf.scale, 0.0f } ),
				b2_colorSaddleBrown );
		}
	}

	void DrawTrees()
	{
		for ( const TreeInstance& tree : m_trees )
		{
			if ( IsXVisible( tree.position.x, 3.0f ) == false )
			{
				continue;
			}

			b2Vec2 trunkVertices[4] = {
				{ -tree.trunkHalfWidth, 0.0f },
				{ tree.trunkHalfWidth, 0.0f },
				{ tree.trunkHalfWidth, tree.trunkHeight },
				{ -tree.trunkHalfWidth, tree.trunkHeight },
			};

			b2Transform trunkTransform = { tree.position, b2MakeRot( 0.0f ) };
			DrawSolidPolygon( m_draw, trunkTransform, trunkVertices, 4, 0.0f, b2_colorSaddleBrown );

			float foliageBaseY = tree.position.y + tree.trunkHeight;

			b2Vec2 foliage1[3] = {
				{ -0.55f * tree.foliageWidth, 0.0f },
				{ 0.55f * tree.foliageWidth, 0.0f },
				{ 0.0f, 0.55f * tree.foliageHeight },
			};
			b2Vec2 foliage2[3] = {
				{ -0.42f * tree.foliageWidth, 0.35f * tree.foliageHeight },
				{ 0.42f * tree.foliageWidth, 0.35f * tree.foliageHeight },
				{ 0.0f, 0.82f * tree.foliageHeight },
			};
			b2Vec2 foliage3[3] = {
				{ -0.30f * tree.foliageWidth, 0.65f * tree.foliageHeight },
				{ 0.30f * tree.foliageWidth, 0.65f * tree.foliageHeight },
				{ 0.0f, tree.foliageHeight },
			};

			b2Transform foliageTransform = { { tree.position.x, foliageBaseY }, b2MakeRot( 0.0f ) };
			DrawSolidPolygon( m_draw, foliageTransform, foliage1, 3, 0.0f, b2_colorDarkGreen );
			DrawSolidPolygon( m_draw, foliageTransform, foliage2, 3, 0.0f, b2_colorForestGreen );
			DrawSolidPolygon( m_draw, foliageTransform, foliage3, 3, 0.0f, b2_colorGreen );
		}
	}

	void DrawFlowers()
	{
		for ( const FlowerInstance& flower : m_flowers )
		{
			if ( IsXVisible( flower.position.x, 1.0f ) == false )
			{
				continue;
			}

			b2Vec2 stemP1 = { flower.position.x, flower.position.y + 0.02f };
			b2Vec2 stemP2 = { flower.position.x, flower.position.y + flower.stemHeight };
			DrawSolidCapsule( m_draw, stemP1, stemP2, 0.025f, b2_colorForestGreen );

			b2Vec2 leaf1[3] = {
				{ 0.0f, 0.18f },
				{ -0.16f, 0.12f },
				{ -0.03f, 0.04f },
			};
			b2Vec2 leaf2[3] = {
				{ 0.0f, 0.24f },
				{ 0.16f, 0.18f },
				{ 0.03f, 0.08f },
			};
			b2Transform stemTransform = { flower.position, b2MakeRot( 0.0f ) };
			DrawSolidPolygon( m_draw, stemTransform, leaf1, 3, 0.0f, b2_colorGreen );
			DrawSolidPolygon( m_draw, stemTransform, leaf2, 3, 0.0f, b2_colorGreen );

			b2Vec2 blossomCenter = { flower.position.x, flower.position.y + flower.stemHeight };
			DrawCircle( m_draw, blossomCenter, 1.8f * flower.petalRadius, b2_colorWhite );
			DrawSolidCircle( m_draw, { blossomCenter, b2MakeRot( 0.0f ) }, flower.petalRadius, flower.petalColor );
			DrawSolidCircle( m_draw, { { blossomCenter.x - 1.4f * flower.petalRadius, blossomCenter.y }, b2MakeRot( 0.0f ) },
				flower.petalRadius, flower.petalColor );
			DrawSolidCircle( m_draw, { { blossomCenter.x + 1.4f * flower.petalRadius, blossomCenter.y }, b2MakeRot( 0.0f ) },
				flower.petalRadius, flower.petalColor );
			DrawSolidCircle( m_draw, { { blossomCenter.x, blossomCenter.y + 1.4f * flower.petalRadius }, b2MakeRot( 0.0f ) },
				flower.petalRadius, flower.petalColor );
			DrawSolidCircle( m_draw, { { blossomCenter.x, blossomCenter.y - 1.4f * flower.petalRadius }, b2MakeRot( 0.0f ) },
				flower.petalRadius, flower.petalColor );
			DrawSolidCircle( m_draw, { blossomCenter, b2MakeRot( 0.0f ) }, 0.65f * flower.petalRadius, b2_colorYellow );
		}
	}

	// ©¤©¤©¤ Skier: compound body with torso, head, arms, legs, and ski board ©¤©¤©¤
	void CreateSkier( SkierRig& skier, b2Vec2 position, bool isAi )
	{
		// We build the skier as multiple bodies connected by revolute joints.
		// The "hip" body is the root; we attach torso, head, arms, legs, and ski.

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.2f;

		// Use a negative group index so skier parts don't collide with each other
		shapeDef.filter.groupIndex = -1;

		float drawSize = 0.05f;

		// ©¤©¤ Torso (root body) ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = position;
			bodyDef.linearDamping = 0.1f; // air resistance
			bodyDef.name = isAi ? "ai_torso" : "torso";
			skier.torsoId = b2CreateBody( m_worldId, &bodyDef );

			// Torso: rectangle 0.3 wide x 0.8 tall
			b2Polygon torso = b2MakeBox( 0.15f, 0.4f );
			shapeDef.material.customColor = isAi ? b2_colorCrimson : b2_colorDodgerBlue;
			b2CreatePolygonShape( skier.torsoId, &shapeDef, &torso );

			// Lower the center of mass for stability
			float mass = b2Body_GetMass( skier.torsoId );
			float inertia = b2Body_GetRotationalInertia( skier.torsoId );
			float offset = 0.2f;
			// Parallel axis theorem: I' = I + m*d^2
			inertia += mass * offset * offset;
			b2MassData massData = { mass, { 0.0f, -offset }, inertia };
			b2Body_SetMassData( skier.torsoId, massData );
		}

		// ©¤©¤ Head ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, 0.55f } );
			bodyDef.name = isAi ? "ai_head" : "head";
			skier.headId = b2CreateBody( m_worldId, &bodyDef );

			b2Circle head = { { 0.0f, 0.0f }, 0.15f };
			shapeDef.material.customColor = b2_colorNavajoWhite;
			b2CreateCircleShape( skier.headId, &shapeDef, &head );

			// Revolute joint: torso <-> head
			b2Vec2 pivot = b2Add( position, { 0.0f, 0.4f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = skier.torsoId;
			jd.base.bodyIdB = skier.headId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( skier.torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( skier.headId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.2f * B2_PI;
			jd.upperAngle = 0.2f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 2.0f;
			jd.base.drawScale = drawSize;
			skier.headJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Left arm (back arm in side view) ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, 0.15f } );
			bodyDef.name = isAi ? "ai_left_arm" : "left_arm";
			skier.leftArmId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon arm = b2MakeBox( 0.05f, 0.2f );
			shapeDef.material.customColor = b2_colorMediumTurquoise;
			b2CreatePolygonShape( skier.leftArmId, &shapeDef, &arm );

			b2Vec2 pivot = b2Add( position, { 0.0f, 0.35f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = skier.torsoId;
			jd.base.bodyIdB = skier.leftArmId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( skier.torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( skier.leftArmId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.4f * B2_PI;
			jd.upperAngle = 0.4f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 1.0f;
			jd.base.drawScale = drawSize;
			skier.leftArmJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Right arm (front arm in side view) ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, 0.15f } );
			bodyDef.name = isAi ? "ai_right_arm" : "right_arm";
			skier.rightArmId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon arm = b2MakeBox( 0.05f, 0.2f );
			shapeDef.material.customColor = b2_colorMediumTurquoise;
			b2CreatePolygonShape( skier.rightArmId, &shapeDef, &arm );

			b2Vec2 pivot = b2Add( position, { 0.0f, 0.35f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = skier.torsoId;
			jd.base.bodyIdB = skier.rightArmId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( skier.torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( skier.rightArmId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.4f * B2_PI;
			jd.upperAngle = 0.4f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 1.0f;
			jd.base.drawScale = drawSize;
			skier.rightArmJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Left leg ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, -0.65f } );
			bodyDef.name = isAi ? "ai_left_leg" : "left_leg";
			skier.leftLegId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon leg = b2MakeBox( 0.06f, 0.25f );
			shapeDef.material.customColor = isAi ? b2_colorCrimson : b2_colorDodgerBlue;
			b2CreatePolygonShape( skier.leftLegId, &shapeDef, &leg );

			b2Vec2 pivot = b2Add( position, { 0.0f, -0.4f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = skier.torsoId;
			jd.base.bodyIdB = skier.leftLegId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( skier.torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( skier.leftLegId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.15f * B2_PI;
			jd.upperAngle = 0.3f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 3.0f;
			jd.base.drawScale = drawSize;
			skier.leftLegJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Right leg ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, -0.65f } );
			bodyDef.name = isAi ? "ai_right_leg" : "right_leg";
			skier.rightLegId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon leg = b2MakeBox( 0.06f, 0.25f );
			shapeDef.material.customColor = isAi ? b2_colorCrimson : b2_colorDodgerBlue;
			b2CreatePolygonShape( skier.rightLegId, &shapeDef, &leg );

			b2Vec2 pivot = b2Add( position, { 0.0f, -0.4f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = skier.torsoId;
			jd.base.bodyIdB = skier.rightLegId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( skier.torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( skier.rightLegId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.15f * B2_PI;
			jd.upperAngle = 0.3f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 3.0f;
			jd.base.drawScale = drawSize;
			skier.rightLegJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Ski board: welded to the bottom of the legs ©¤©¤
		// The ski is a long thin box, welded to the torso at the foot position.
		// Using a weld joint with some angular compliance for slight flex.
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, -0.95f } );
			bodyDef.name = isAi ? "ai_ski" : "ski";
			skier.skiId = b2CreateBody( m_worldId, &bodyDef );

			// Ski board: 1.2 long x 0.15 tall (half-extents: 0.6 x 0.075)
			b2Polygon skiBox = b2MakeBox( 0.6f, 0.075f );
			b2ShapeDef skiShapeDef = b2DefaultShapeDef();
			skiShapeDef.density = 0.5f;
			skiShapeDef.material.friction = 0.03f; // very low friction for gliding
			skiShapeDef.material.customColor = isAi ? b2_colorDarkOrange : b2_colorSaddleBrown;
			skiShapeDef.filter.groupIndex = -1;
			b2CreatePolygonShape( skier.skiId, &skiShapeDef, &skiBox );

			// Weld the ski to the torso at the foot attachment point
			b2Vec2 weldPoint = b2Add( position, { 0.0f, -0.9f } );
			b2WeldJointDef wd = b2DefaultWeldJointDef();
			wd.base.bodyIdA = skier.torsoId;
			wd.base.bodyIdB = skier.skiId;
			wd.base.localFrameA.p = b2Body_GetLocalPoint( skier.torsoId, weldPoint );
			wd.base.localFrameB.p = b2Body_GetLocalPoint( skier.skiId, weldPoint );
			// Slight angular compliance for natural flex
			wd.angularHertz = 8.0f;
			wd.angularDampingRatio = 1.0f;
			wd.linearHertz = 0.0f; // rigid linear
			wd.linearDampingRatio = 0.0f;
			wd.base.drawScale = drawSize;
			skier.skiWeldJointId = b2CreateWeldJoint( m_worldId, &wd );
		}
	}

	bool IsAirborne( const SkierRig& skier ) const
	{
		int contactCapacity = b2Body_GetContactCapacity( skier.skiId );
		if ( contactCapacity <= 0 )
		{
			return true;
		}

		constexpr int maxContacts = 8;
		b2ContactData contactData[maxContacts];
		int contactCount = b2Body_GetContactData( skier.skiId, contactData, maxContacts );
		for ( int i = 0; i < contactCount; ++i )
		{
			if ( contactData[i].manifold.pointCount > 0 )
			{
				return false;
			}
		}

		return true;
	}

	b2BodyId FindBlockingRock( const SkierRig& skier ) const
	{
		b2Vec2 aiPos = b2Body_GetPosition( skier.torsoId );
		b2BodyId bestRockId = b2_nullBodyId;
		float bestDx = 1.0e30f;

		for ( b2BodyId rockId : m_rockIds )
		{
			b2Vec2 rockPos = b2Body_GetPosition( rockId );
			float dx = rockPos.x - aiPos.x;
			if ( dx < 0.8f || dx > 6.0f )
			{
				continue;
			}

			float terrainY = GetTerrainHeight( rockPos.x );
			if ( rockPos.y - terrainY > m_aiJumpHeightTolerance && dx < bestDx )
			{
				bestDx = dx;
				bestRockId = rockId;
			}
		}

		return bestRockId;
	}

	void ResetSkierPose( SkierRig& skier, b2Vec2 position )
	{
		b2Rot upright = b2MakeRot( 0.0f );
		b2Body_SetTransform( skier.torsoId, position, upright );
		b2Body_SetTransform( skier.headId, b2Add( position, { 0.0f, 0.55f } ), upright );
		b2Body_SetTransform( skier.leftArmId, b2Add( position, { 0.0f, 0.15f } ), upright );
		b2Body_SetTransform( skier.rightArmId, b2Add( position, { 0.0f, 0.15f } ), upright );
		b2Body_SetTransform( skier.leftLegId, b2Add( position, { 0.0f, -0.65f } ), upright );
		b2Body_SetTransform( skier.rightLegId, b2Add( position, { 0.0f, -0.65f } ), upright );
		b2Body_SetTransform( skier.skiId, b2Add( position, { 0.0f, -0.95f } ), upright );

		b2Body_SetLinearVelocity( skier.torsoId, { 0.0f, 0.0f } );
		b2Body_SetLinearVelocity( skier.headId, { 0.0f, 0.0f } );
		b2Body_SetLinearVelocity( skier.leftArmId, { 0.0f, 0.0f } );
		b2Body_SetLinearVelocity( skier.rightArmId, { 0.0f, 0.0f } );
		b2Body_SetLinearVelocity( skier.leftLegId, { 0.0f, 0.0f } );
		b2Body_SetLinearVelocity( skier.rightLegId, { 0.0f, 0.0f } );
		b2Body_SetLinearVelocity( skier.skiId, { 0.0f, 0.0f } );

		b2Body_SetAngularVelocity( skier.torsoId, 0.0f );
		b2Body_SetAngularVelocity( skier.headId, 0.0f );
		b2Body_SetAngularVelocity( skier.leftArmId, 0.0f );
		b2Body_SetAngularVelocity( skier.rightArmId, 0.0f );
		b2Body_SetAngularVelocity( skier.leftLegId, 0.0f );
		b2Body_SetAngularVelocity( skier.rightLegId, 0.0f );
		b2Body_SetAngularVelocity( skier.skiId, 0.0f );

		skier.prevJumpPressed = false;
		skier.jumpBufferSteps = 0;
		skier.coyoteSteps = m_coyoteDuration;
		skier.reverseSteps = 0;
		skier.recoveryJumpArmed = true;
		skier.stuckSteps = 0;
		skier.boostTime = 0.0f;
		skier.parachuteTime = 0.0f;
	}

	void UpdateAiRecovery( SkierRig& skier )
	{
		b2Vec2 aiVelocity = b2Body_GetLinearVelocity( skier.torsoId );
		b2Rot aiRotation = b2Body_GetRotation( skier.torsoId );
		bool tiltedOver = aiRotation.c < m_aiRespawnTiltCos;
		bool nearBlockedRock = B2_IS_NON_NULL( FindBlockingRock( skier ) );

		if ( nearBlockedRock && ( tiltedOver || aiVelocity.x < 0.5f ) )
		{
			skier.stuckSteps += 1;
		}
		else
		{
			skier.stuckSteps = 0;
		}

		if ( skier.stuckSteps < m_aiRespawnStuckSteps )
		{
			return;
		}

		b2BodyId rockId = FindBlockingRock( skier );
		if ( B2_IS_NULL( rockId ) )
		{
			skier.stuckSteps = 0;
			return;
		}

		b2Vec2 rockPos = b2Body_GetPosition( rockId );
		float resetX = rockPos.x - 4.5f;
		float resetY = GetTerrainHeight( resetX ) + 1.35f;
		ResetSkierPose( skier, { resetX, resetY } );
	}

	bool IsAiBlockedByRock( const SkierRig& skier ) const
	{
		b2Vec2 aiPos = b2Body_GetPosition( skier.torsoId );
		b2Vec2 aiVel = b2Body_GetLinearVelocity( skier.torsoId );

		if ( aiVel.x > m_aiStuckSpeedThreshold )
		{
			return false;
		}

		for ( b2BodyId rockId : m_rockIds )
		{
			b2Vec2 rockPos = b2Body_GetPosition( rockId );
			float dx = rockPos.x - aiPos.x;
			if ( dx < 0.8f || dx > 4.0f )
			{
				continue;
			}

			float terrainY = GetTerrainHeight( rockPos.x );
			if ( rockPos.y - terrainY > m_aiJumpHeightTolerance )
			{
				return true;
			}
		}

		return false;
	}

	bool ShouldAiJump( const SkierRig& skier ) const
	{
		b2Vec2 aiPos = b2Body_GetPosition( skier.torsoId );
		b2Vec2 aiVel = b2Body_GetLinearVelocity( skier.torsoId );
		float lookAhead = m_aiJumpLookAhead + 0.2f * b2AbsFloat( aiVel.x );
		lookAhead = b2ClampFloat( lookAhead, 4.0f, 10.0f );

		for ( b2BodyId rockId : m_rockIds )
		{
			b2Vec2 rockPos = b2Body_GetPosition( rockId );
			float dx = rockPos.x - aiPos.x;
			if ( dx < 1.4f || dx > lookAhead )
			{
				continue;
			}

			float terrainY = GetTerrainHeight( rockPos.x );
			float obstacleHeight = rockPos.y - terrainY;
			if ( obstacleHeight > m_aiJumpHeightTolerance && dx < 3.8f + 0.08f * b2AbsFloat( aiVel.x ) )
			{
				return true;
			}
		}

		return false;
	}

	const RocketPickup* FindAiTargetRocket( const SkierRig& skier ) const
	{
		b2Vec2 aiPos = b2Body_GetPosition( skier.torsoId );
		const RocketPickup* bestRocket = nullptr;
		float bestScore = 1.0e30f;

		for ( const RocketPickup& rocket : m_rocketPickups )
		{
			if ( rocket.collected )
			{
				continue;
			}

			float dx = rocket.position.x - aiPos.x;
			if ( dx < 1.0f || dx > m_aiRocketSeekRange )
			{
				continue;
			}

			float dy = rocket.position.y - aiPos.y;
			if ( b2AbsFloat( dy ) > m_aiRocketSeekHeightTolerance )
			{
				continue;
			}

			float score = dx + 0.75f * b2AbsFloat( dy );
			if ( score < bestScore )
			{
				bestScore = score;
				bestRocket = &rocket;
			}
		}

		return bestRocket;
	}

	void StepSkier( SkierRig& skier, bool moveRight, bool moveLeft, bool jumpPressed, bool brakeOnly = false )
	{
		float speedMultiplier = skier.boostTime > 0.0f ? m_boostSpeedMultiplier : 1.0f;
		float maxAllowedSpeed = m_maxSpeed * speedMultiplier;
		float appliedForce = m_forceStrength * speedMultiplier;
		b2Vec2 velocity = b2Body_GetLinearVelocity( skier.torsoId );
		bool airborne = IsAirborne( skier );

		if ( moveRight && velocity.x < maxAllowedSpeed )
		{
			b2Body_ApplyForceToCenter( skier.torsoId, { appliedForce, 0.0f }, true );
		}

		if ( airborne == false && moveLeft )
		{
			if ( brakeOnly )
			{
				if ( velocity.x > 0.0f )
				{
					b2Body_ApplyForceToCenter( skier.torsoId, { -appliedForce, 0.0f }, true );
				}
			}
			else if ( velocity.x > -maxAllowedSpeed )
			{
				b2Body_ApplyForceToCenter( skier.torsoId, { -appliedForce, 0.0f }, true );
			}
		}

		if ( airborne == false )
		{
			skier.coyoteSteps = m_coyoteDuration;
		}
		else if ( skier.coyoteSteps > 0 )
		{
			skier.coyoteSteps -= 1;
		}

		if ( jumpPressed && skier.prevJumpPressed == false )
		{
			skier.jumpBufferSteps = m_jumpBufferDuration;
		}
		else if ( skier.jumpBufferSteps > 0 )
		{
			skier.jumpBufferSteps -= 1;
		}

		bool canJump = airborne == false || skier.coyoteSteps > 0;
		if ( skier.jumpBufferSteps > 0 && canJump )
		{
			b2Body_ApplyLinearImpulseToCenter( skier.torsoId, { 0.0f, m_jumpImpulse }, true );
			b2Body_ApplyLinearImpulseToCenter( skier.skiId, { 0.0f, 0.5f * m_jumpImpulse }, true );
			airborne = true;
			skier.jumpBufferSteps = 0;
			skier.coyoteSteps = 0;

			if ( &skier != &m_player )
			{
				skier.recoveryJumpArmed = false;
			}
		}

		skier.prevJumpPressed = jumpPressed;

		b2Rot rotation = b2Body_GetRotation( skier.torsoId );
		float angle = b2Rot_GetAngle( rotation );
		float angularVel = b2Body_GetAngularVelocity( skier.torsoId );

		if ( airborne )
		{
			if ( skier.parachuteTime > 0.0f )
			{
				constexpr float liftRatio = 0.8f;
				constexpr float gravityMagnitude = 9.8f;
				b2BodyId bodyIds[] = {
					skier.torsoId,
					skier.headId,
					skier.leftArmId,
					skier.rightArmId,
					skier.leftLegId,
					skier.rightLegId,
					skier.skiId,
				};

				for ( b2BodyId bodyId : bodyIds )
				{
					float liftForce = liftRatio * gravityMagnitude * b2Body_GetMass( bodyId );
					b2Body_ApplyForceToCenter( bodyId, { 0.0f, liftForce }, true );
				}
			}

			float corrective = -m_airBalanceStrength * angle - m_airDampingStrength * angularVel;
			b2Body_ApplyTorque( skier.torsoId, corrective, true );
		}
		else
		{
			if ( angle > m_balanceAngleLimit )
			{
				b2Body_ApplyTorque( skier.torsoId, -m_balanceTorque, true );
			}
			else if ( angle < -m_balanceAngleLimit )
			{
				b2Body_ApplyTorque( skier.torsoId, m_balanceTorque, true );
			}
		}
	}

	void UpdateGui() override
	{
		int playerRank = 1;
		float playerX = b2Body_GetPosition( m_player.torsoId ).x;
		for ( const SkierRig& skier : m_aiSkiers )
		{
			if ( b2Body_GetPosition( skier.torsoId ).x > playerX )
			{
				playerRank += 1;
			}
		}

		float distanceToFinish = b2MaxFloat( 0.0f, ( GetTrackEndX() - 6.0f ) - playerX );

		ImGuiWindowFlags overlayFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
			ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_AlwaysAutoResize |
			ImGuiWindowFlags_NoInputs;
		ImGui::SetNextWindowBgAlpha( 0.30f );
		ImGui::SetNextWindowPos( ImVec2( 0.5f * m_camera->width, 18.0f ), ImGuiCond_Always, ImVec2( 0.5f, 0.0f ) );
		if ( ImGui::Begin( "SkiingStatsOverlay", nullptr, overlayFlags ) )
		{
			ImGui::Text( "FPS: %.1f", m_context->averageFps );
			ImGui::Text( "Rank: %d/%d", playerRank, m_aiCount + 1 );
			ImGui::Text( "Distance to Finish: %.1f m", distanceToFinish );
		}
		ImGui::End();

		float fontSize = ImGui::GetFontSize();
		float height = 300.0f;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 280.0f, height ) );

		ImGui::Begin( "Skiing", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::SliderFloat( "Force", &m_forceStrength, 10.0f, 200.0f, "%.0f N" );
		ImGui::SliderFloat( "Max Speed", &m_maxSpeed, 5.0f, 30.0f, "%.0f m/s" );
		ImGui::SliderFloat( "Jump Impulse", &m_jumpImpulse, 1.0f, 12.0f, "%.1f N*s" );
		ImGui::SliderFloat( "Balance Torque", &m_balanceTorque, 0.0f, 30.0f, "%.0f N*m" );

		float angleDeg = m_balanceAngleLimit * 180.0f / B2_PI;
		if ( ImGui::SliderFloat( "Tilt Limit", &angleDeg, 5.0f, 45.0f, "%.0f deg" ) )
		{
			m_balanceAngleLimit = angleDeg * B2_PI / 180.0f;
		}

		ImGui::SliderFloat( "Air Balance", &m_airBalanceStrength, 0.0f, 50.0f, "%.0f" );
		ImGui::SliderFloat( "Air Damping", &m_airDampingStrength, 0.0f, 20.0f, "%.0f" );

		ImGui::Checkbox( "Follow Camera", &m_followCamera );
		if ( ImGui::Button( "Reset Game" ) )
		{
			ResetGame();
		}
		if ( ImGui::Button( "Follow Next AI" ) )
		{
			FocusNextAi();
		}
		if ( ImGui::Button( "Follow Player" ) )
		{
			FocusPlayer();
		}

		b2Vec2 vel = b2Body_GetLinearVelocity( m_player.torsoId );
		float speed = b2Length( vel );
		float minAiSpeed = 1.0e30f;
		float maxAiSpeed = 0.0f;
		for ( const SkierRig& skier : m_aiSkiers )
		{
			float aiSpeed = b2Length( b2Body_GetLinearVelocity( skier.torsoId ) );
			minAiSpeed = b2MinFloat( minAiSpeed, aiSpeed );
			maxAiSpeed = b2MaxFloat( maxAiSpeed, aiSpeed );
		}
		ImGui::Text( "Speed: %.1f m/s", speed );
		ImGui::Text( "AI Count: %d", m_aiCount );
		ImGui::Text( "Fastest AI: %.1f m/s", maxAiSpeed );
		ImGui::Text( "Slowest AI: %.1f m/s", minAiSpeed );
		ImGui::Text( "Press D to accelerate right" );
		ImGui::Text( "Press S to slow down" );
		ImGui::Text( "Press A to accelerate left" );
		ImGui::Text( "Press Space to jump" );
		ImGui::Text( "Red skier is AI-controlled" );
		ImGui::Text( "Small rocks are scattered on the slope" );
		if ( m_cameraTargetIndex < 0 )
		{
			ImGui::Text( "Camera: Player" );
		}
		else
		{
			ImGui::Text( "Camera: AI %d", m_cameraTargetIndex + 1 );
		}

		ImGui::End();
	}

	void Step() override
	{
		// ©¤©¤ Keyboard input: apply horizontal force ©¤©¤
		// Poll the D key for continuous acceleration
		bool pushRight = glfwGetKey( m_context->window, GLFW_KEY_D ) == GLFW_PRESS;
		bool slowDown = glfwGetKey( m_context->window, GLFW_KEY_S ) == GLFW_PRESS;
		bool pushLeft = glfwGetKey( m_context->window, GLFW_KEY_A ) == GLFW_PRESS;
		bool jumpPressed = glfwGetKey( m_context->window, GLFW_KEY_SPACE ) == GLFW_PRESS;

		b2Vec2 velocity = b2Body_GetLinearVelocity( m_player.torsoId );
		float speed = b2Length( velocity );

		UpdateBoosts();

		StepSkier( m_player, pushRight, pushLeft || slowDown, jumpPressed, slowDown && pushLeft == false );
		for ( SkierRig& skier : m_aiSkiers )
		{
			StepAi( skier );
		}

		UpdateRocketPickups();
		UpdateParachutePickups();

		// ©¤©¤ Follow camera ©¤©¤
		if ( m_followCamera )
		{
			const SkierRig& cameraTarget = GetCameraTarget();
			b2Vec2 pos = b2Body_GetPosition( cameraTarget.torsoId );
			m_context->camera.center = { pos.x + 5.0f, pos.y + 3.0f };
		}

		DrawTrees();
		DrawClouds();
		DrawFlowers();
		DrawFallenLeaves();
		DrawRocketPickups();
		DrawParachutePickups();
		DrawBoostEffect( m_player );
		DrawParachuteEffect( m_player );
		for ( const SkierRig& skier : m_aiSkiers )
		{
			DrawBoostEffect( skier );
			DrawParachuteEffect( skier );
		}

		Sample::Step();

		// ©¤©¤ HUD overlay ©¤©¤
		DrawTextLine( "Skiing Demo - Use A/D keys to control, gravity-driven descent" );

		if ( speed < 0.1f && !pushRight && !pushLeft )
		{
			DrawTextLine( "Skier is nearly stopped" );
		}
	}

	void StepAi( SkierRig& skier )
	{
		UpdateAiRecovery( skier );

		b2Vec2 aiVelocity = b2Body_GetLinearVelocity( skier.torsoId );
		b2Vec2 aiPos = b2Body_GetPosition( skier.torsoId );
		const RocketPickup* targetRocket = FindAiTargetRocket( skier );
		if ( skier.reverseSteps == 0 && skier.recoveryJumpArmed == false && IsAiBlockedByRock( skier ) )
		{
			skier.reverseSteps = m_aiReverseDuration;
			skier.recoveryJumpArmed = true;
		}

		bool aiPushRight = false;
		bool aiPushLeft = false;
		bool aiJumpPressed = false;

		if ( skier.reverseSteps > 0 )
		{
			aiPushLeft = true;
			skier.reverseSteps -= 1;
		}
		else
		{
			float targetCruiseSpeed = skier.cruiseSpeed;
			if ( targetRocket != nullptr )
			{
				targetCruiseSpeed = b2MaxFloat( targetCruiseSpeed, m_aiMaxCruiseSpeed );
			}

			aiPushRight = aiVelocity.x < targetCruiseSpeed;
			aiJumpPressed = ShouldAiJump( skier );
			if ( targetRocket != nullptr )
			{
				float rocketDx = targetRocket->position.x - aiPos.x;
				float rocketDy = targetRocket->position.y - aiPos.y;
				if ( rocketDx > 1.5f && rocketDx < 8.5f && rocketDy > m_aiRocketJumpHeight )
				{
					aiJumpPressed = true;
				}
			}

			if ( skier.recoveryJumpArmed )
			{
				aiJumpPressed = true;
			}
		}

		StepSkier( skier, aiPushRight, aiPushLeft, aiJumpPressed );
	}

	const SkierRig& GetCameraTarget() const
	{
		if ( m_cameraTargetIndex >= 0 && m_cameraTargetIndex < m_aiCount )
		{
			return m_aiSkiers[m_cameraTargetIndex];
		}

		return m_player;
	}

	void FocusNextAi()
	{
		m_cameraTargetIndex = ( m_cameraTargetIndex + 1 ) % m_aiCount;
		m_followCamera = true;
	}

	void FocusPlayer()
	{
		m_cameraTargetIndex = -1;
		m_followCamera = true;
	}

	void ResetGame()
	{
		ResetSkierPose( m_player, { -45.0f, 22.0f } );

		for ( int i = 0; i < m_aiCount; ++i )
		{
			float x = -42.0f + 3.0f * i;
			float y = 24.0f + 1.3f * float( i % 6 );
			ResetSkierPose( m_aiSkiers[i], { x, y } );
		}

		CreateRocketPickups();
		CreateParachutePickups();
		FocusPlayer();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Skiing( context );
	}

	// Ground
	b2BodyId m_groundId;
	b2ChainId m_chainId;
	b2BodyId m_finishId;
	b2BodyId m_castleId;

	SkierRig m_player;
	static constexpr int m_aiCount = 19;
	std::array<SkierRig, m_aiCount> m_aiSkiers;
	std::vector<b2BodyId> m_rockIds;
	std::vector<RocketPickup> m_rocketPickups;
	std::vector<ParachutePickup> m_parachutePickups;
	std::vector<CloudInstance> m_clouds;
	std::vector<LeafInstance> m_leaves;
	std::vector<TreeInstance> m_trees;
	std::vector<FlowerInstance> m_flowers;

	// Tuning parameters
	float m_forceStrength;
	float m_maxSpeed;
	float m_jumpImpulse;
	float m_boostDuration;
	float m_boostSpeedMultiplier;
	float m_parachuteDuration;
	int m_jumpBufferDuration;
	int m_coyoteDuration;
	float m_aiCruiseSpeed;
	float m_aiJumpLookAhead;
	float m_aiJumpHeightTolerance;
	float m_aiRocketSeekRange;
	float m_aiRocketSeekHeightTolerance;
	float m_aiRocketJumpHeight;
	float m_aiMaxCruiseSpeed;
	int m_aiReverseDuration;
	float m_aiStuckSpeedThreshold;
	float m_aiRespawnTiltCos;
	int m_aiRespawnStuckSteps;
	float m_balanceTorque;
	float m_balanceAngleLimit;
	float m_airBalanceStrength;
	float m_airDampingStrength;
	bool m_followCamera;
	int m_cameraTargetIndex;
	static constexpr int m_trackSegCount = 3200;
};

static int sampleSkiing = RegisterSample( "Shapes", "Skiing", Skiing::Create );
