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

class Skiing : public Sample
{
public:
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
		CreateSkier( { -45.0f, 22.0f } );

		m_forceStrength = 50.0f;
		m_maxSpeed = 15.0f;
		m_balanceTorque = 10.0f;
		m_balanceAngleLimit = 15.0f * B2_PI / 180.0f;
		m_airBalanceStrength = 20.0f;
		m_airDampingStrength = 5.0f;
		m_followCamera = true;
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
		constexpr int segCount = 800;
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
	}

	// ©¤©¤©¤ Skier: compound body with torso, head, arms, legs, and ski board ©¤©¤©¤
	void CreateSkier( b2Vec2 position )
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
			bodyDef.name = "torso";
			m_torsoId = b2CreateBody( m_worldId, &bodyDef );

			// Torso: rectangle 0.3 wide x 0.8 tall
			b2Polygon torso = b2MakeBox( 0.15f, 0.4f );
			shapeDef.material.customColor = b2_colorDodgerBlue;
			b2CreatePolygonShape( m_torsoId, &shapeDef, &torso );

			// Lower the center of mass for stability
			float mass = b2Body_GetMass( m_torsoId );
			float inertia = b2Body_GetRotationalInertia( m_torsoId );
			float offset = 0.2f;
			// Parallel axis theorem: I' = I + m*d^2
			inertia += mass * offset * offset;
			b2MassData massData = { mass, { 0.0f, -offset }, inertia };
			b2Body_SetMassData( m_torsoId, massData );
		}

		// ©¤©¤ Head ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, 0.55f } );
			bodyDef.name = "head";
			m_headId = b2CreateBody( m_worldId, &bodyDef );

			b2Circle head = { { 0.0f, 0.0f }, 0.15f };
			shapeDef.material.customColor = b2_colorNavajoWhite;
			b2CreateCircleShape( m_headId, &shapeDef, &head );

			// Revolute joint: torso <-> head
			b2Vec2 pivot = b2Add( position, { 0.0f, 0.4f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = m_torsoId;
			jd.base.bodyIdB = m_headId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( m_torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( m_headId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.2f * B2_PI;
			jd.upperAngle = 0.2f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 2.0f;
			jd.base.drawScale = drawSize;
			m_headJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Left arm (back arm in side view) ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, 0.15f } );
			bodyDef.name = "left_arm";
			m_leftArmId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon arm = b2MakeBox( 0.05f, 0.2f );
			shapeDef.material.customColor = b2_colorMediumTurquoise;
			b2CreatePolygonShape( m_leftArmId, &shapeDef, &arm );

			b2Vec2 pivot = b2Add( position, { 0.0f, 0.35f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = m_torsoId;
			jd.base.bodyIdB = m_leftArmId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( m_torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( m_leftArmId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.4f * B2_PI;
			jd.upperAngle = 0.4f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 1.0f;
			jd.base.drawScale = drawSize;
			m_leftArmJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Right arm (front arm in side view) ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, 0.15f } );
			bodyDef.name = "right_arm";
			m_rightArmId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon arm = b2MakeBox( 0.05f, 0.2f );
			shapeDef.material.customColor = b2_colorMediumTurquoise;
			b2CreatePolygonShape( m_rightArmId, &shapeDef, &arm );

			b2Vec2 pivot = b2Add( position, { 0.0f, 0.35f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = m_torsoId;
			jd.base.bodyIdB = m_rightArmId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( m_torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( m_rightArmId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.4f * B2_PI;
			jd.upperAngle = 0.4f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 1.0f;
			jd.base.drawScale = drawSize;
			m_rightArmJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Left leg ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, -0.65f } );
			bodyDef.name = "left_leg";
			m_leftLegId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon leg = b2MakeBox( 0.06f, 0.25f );
			shapeDef.material.customColor = b2_colorDodgerBlue;
			b2CreatePolygonShape( m_leftLegId, &shapeDef, &leg );

			b2Vec2 pivot = b2Add( position, { 0.0f, -0.4f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = m_torsoId;
			jd.base.bodyIdB = m_leftLegId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( m_torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( m_leftLegId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.15f * B2_PI;
			jd.upperAngle = 0.3f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 3.0f;
			jd.base.drawScale = drawSize;
			m_leftLegJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Right leg ©¤©¤
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, -0.65f } );
			bodyDef.name = "right_leg";
			m_rightLegId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon leg = b2MakeBox( 0.06f, 0.25f );
			shapeDef.material.customColor = b2_colorDodgerBlue;
			b2CreatePolygonShape( m_rightLegId, &shapeDef, &leg );

			b2Vec2 pivot = b2Add( position, { 0.0f, -0.4f } );
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.base.bodyIdA = m_torsoId;
			jd.base.bodyIdB = m_rightLegId;
			jd.base.localFrameA.p = b2Body_GetLocalPoint( m_torsoId, pivot );
			jd.base.localFrameB.p = b2Body_GetLocalPoint( m_rightLegId, pivot );
			jd.enableLimit = true;
			jd.lowerAngle = -0.15f * B2_PI;
			jd.upperAngle = 0.3f * B2_PI;
			jd.enableMotor = true;
			jd.maxMotorTorque = 3.0f;
			jd.base.drawScale = drawSize;
			m_rightLegJointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		// ©¤©¤ Ski board: welded to the bottom of the legs ©¤©¤
		// The ski is a long thin box, welded to the torso at the foot position.
		// Using a weld joint with some angular compliance for slight flex.
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Add( position, { 0.0f, -0.95f } );
			bodyDef.name = "ski";
			m_skiId = b2CreateBody( m_worldId, &bodyDef );

			// Ski board: 1.2 long x 0.15 tall (half-extents: 0.6 x 0.075)
			b2Polygon skiBox = b2MakeBox( 0.6f, 0.075f );
			b2ShapeDef skiShapeDef = b2DefaultShapeDef();
			skiShapeDef.density = 0.5f;
			skiShapeDef.material.friction = 0.03f; // very low friction for gliding
			skiShapeDef.material.customColor = b2_colorSaddleBrown;
			skiShapeDef.filter.groupIndex = -1;
			b2CreatePolygonShape( m_skiId, &skiShapeDef, &skiBox );

			// Weld the ski to the torso at the foot attachment point
			b2Vec2 weldPoint = b2Add( position, { 0.0f, -0.9f } );
			b2WeldJointDef wd = b2DefaultWeldJointDef();
			wd.base.bodyIdA = m_torsoId;
			wd.base.bodyIdB = m_skiId;
			wd.base.localFrameA.p = b2Body_GetLocalPoint( m_torsoId, weldPoint );
			wd.base.localFrameB.p = b2Body_GetLocalPoint( m_skiId, weldPoint );
			// Slight angular compliance for natural flex
			wd.angularHertz = 8.0f;
			wd.angularDampingRatio = 1.0f;
			wd.linearHertz = 0.0f; // rigid linear
			wd.linearDampingRatio = 0.0f;
			wd.base.drawScale = drawSize;
			m_skiWeldJointId = b2CreateWeldJoint( m_worldId, &wd );
		}
	}

	void UpdateGui() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 270.0f;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 280.0f, height ) );

		ImGui::Begin( "Skiing", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::SliderFloat( "Force", &m_forceStrength, 10.0f, 200.0f, "%.0f N" );
		ImGui::SliderFloat( "Max Speed", &m_maxSpeed, 5.0f, 30.0f, "%.0f m/s" );
		ImGui::SliderFloat( "Balance Torque", &m_balanceTorque, 0.0f, 30.0f, "%.0f N*m" );

		float angleDeg = m_balanceAngleLimit * 180.0f / B2_PI;
		if ( ImGui::SliderFloat( "Tilt Limit", &angleDeg, 5.0f, 45.0f, "%.0f deg" ) )
		{
			m_balanceAngleLimit = angleDeg * B2_PI / 180.0f;
		}

		ImGui::SliderFloat( "Air Balance", &m_airBalanceStrength, 0.0f, 50.0f, "%.0f" );
		ImGui::SliderFloat( "Air Damping", &m_airDampingStrength, 0.0f, 20.0f, "%.0f" );

		ImGui::Checkbox( "Follow Camera", &m_followCamera );

		b2Vec2 vel = b2Body_GetLinearVelocity( m_torsoId );
		float speed = b2Length( vel );
		ImGui::Text( "Speed: %.1f m/s", speed );
		ImGui::Text( "Press D to accelerate right" );
		ImGui::Text( "Press A to accelerate left" );

		ImGui::End();
	}

	void Step() override
	{
		// ©¤©¤ Keyboard input: apply horizontal force ©¤©¤
		// Poll the D key for continuous acceleration
		bool pushRight = glfwGetKey( m_context->window, GLFW_KEY_D ) == GLFW_PRESS;
		bool pushLeft = glfwGetKey( m_context->window, GLFW_KEY_A ) == GLFW_PRESS;

		b2Vec2 velocity = b2Body_GetLinearVelocity( m_torsoId );
		float speed = b2Length( velocity );

		if ( pushRight && velocity.x < m_maxSpeed )
		{
			b2Body_ApplyForceToCenter( m_torsoId, { m_forceStrength, 0.0f }, true );
		}

		if ( pushLeft && velocity.x > -m_maxSpeed )
		{
			b2Body_ApplyForceToCenter( m_torsoId, { -m_forceStrength, 0.0f }, true );
		}

		// ©¤©¤ Detect if the skier is airborne ©¤©¤
		// Check for touching contacts on the ski body
		bool airborne = true;
		{
			int contactCapacity = b2Body_GetContactCapacity( m_skiId );
			if ( contactCapacity > 0 )
			{
				constexpr int maxContacts = 8;
				b2ContactData contactData[maxContacts];
				int contactCount = b2Body_GetContactData( m_skiId, contactData, maxContacts );
				for ( int i = 0; i < contactCount; ++i )
				{
					if ( contactData[i].manifold.pointCount > 0 )
					{
						airborne = false;
						break;
					}
				}
			}
		}

		// ©¤©¤ Balance stabilization ©¤©¤
		b2Rot rotation = b2Body_GetRotation( m_torsoId );
		float angle = b2Rot_GetAngle( rotation );
		float angularVel = b2Body_GetAngularVelocity( m_torsoId );

		if ( airborne )
		{
			// Airborne: apply PD controller torque to keep the skier upright
			// and damp angular velocity to prevent spinning.
			float corrective = -m_airBalanceStrength * angle - m_airDampingStrength * angularVel;
			b2Body_ApplyTorque( m_torsoId, corrective, true );
		}
		else
		{
			// On ground: only correct when tilted beyond the angle limit
			if ( angle > m_balanceAngleLimit )
			{
				b2Body_ApplyTorque( m_torsoId, -m_balanceTorque, true );
			}
			else if ( angle < -m_balanceAngleLimit )
			{
				b2Body_ApplyTorque( m_torsoId, m_balanceTorque, true );
			}
		}

		// ©¤©¤ Follow camera ©¤©¤
		if ( m_followCamera )
		{
			b2Vec2 pos = b2Body_GetPosition( m_torsoId );
			m_context->camera.center = { pos.x + 5.0f, pos.y + 3.0f };
		}

		Sample::Step();

		// ©¤©¤ HUD overlay ©¤©¤
		DrawTextLine( "Skiing Demo - Use A/D keys to control, gravity-driven descent" );

		if ( speed < 0.1f && !pushRight && !pushLeft )
		{
			DrawTextLine( "Skier is nearly stopped" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Skiing( context );
	}

	// Ground
	b2BodyId m_groundId;
	b2ChainId m_chainId;

	// Skier body parts
	b2BodyId m_torsoId;
	b2BodyId m_headId;
	b2BodyId m_leftArmId;
	b2BodyId m_rightArmId;
	b2BodyId m_leftLegId;
	b2BodyId m_rightLegId;
	b2BodyId m_skiId;

	// Joints
	b2JointId m_headJointId;
	b2JointId m_leftArmJointId;
	b2JointId m_rightArmJointId;
	b2JointId m_leftLegJointId;
	b2JointId m_rightLegJointId;
	b2JointId m_skiWeldJointId;

	// Tuning parameters
	float m_forceStrength;
	float m_maxSpeed;
	float m_balanceTorque;
	float m_balanceAngleLimit;
	float m_airBalanceStrength;
	float m_airDampingStrength;
	bool m_followCamera;
};

static int sampleSkiing = RegisterSample( "Shapes", "Skiing", Skiing::Create );
