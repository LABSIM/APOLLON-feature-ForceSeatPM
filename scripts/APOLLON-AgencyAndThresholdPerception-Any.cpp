#include "GameSpecific.h"
#include "ScriptsAPI.h"

namespace
{

	float g_filtered_Roll              = 0.0;
	float g_filtered_Pitch             = 0.0;
	float g_filtered_Yaw               = 0.0;
	float g_filtered_RollSpeed         = 0.0;
	float g_filtered_PitchSpeed        = 0.0;
	float g_filtered_YawSpeed          = 0.0;
	float g_filtered_RollAcceleration  = 0.0;
	float g_filtered_PitchAcceleration = 0.0;
	float g_filtered_YawAcceleration   = 0.0;
	float g_filtered_SurgeSpeed        = 0.0;
	float g_filtered_HeaveSpeed        = 0.0;
	float g_filtered_SwaySpeed         = 0.0;
	float g_filtered_SurgeAcceleration = 0.0;
	float g_filtered_HeaveAcceleration = 0.0;
	float g_filtered_SwayAcceleration  = 0.0;

	float g_previous_Roll              = 0.0;
	float g_previous_Pitch             = 0.0;
	float g_previous_Yaw               = 0.0;
	float g_previous_Surge             = 0.0;
	float g_previous_Heave             = 0.0;
	float g_previous_Sway              = 0.0;
	float g_previous_RollSpeed         = 0.0;
	float g_previous_PitchSpeed        = 0.0;
	float g_previous_YawSpeed          = 0.0;
	float g_previous_SurgeSpeed        = 0.0;
	float g_previous_HeaveSpeed        = 0.0;
	float g_previous_SwaySpeed         = 0.0;

} /* } anonymous */

MOSY_SCRIPT_API_EXPORT void setup(MSSA::Context& /*ctx*/, MSSA::System& /*sys*/)
{ /* empty */ }

MOSY_SCRIPT_API_EXPORT void process(MSSA::Context& ctx, MSSA::System& sys)
{

	auto& in         = ctx.input.fromGame.named;
	auto& motionTune = ctx.input.motionTunes.named;
	auto& tabfTune   = ctx.input.tabfTunes.named;

	if (sys.mpu.fsmiHandleParking(ctx))
	{
		return;
	}

	// 1:1 ratio from sim (unfiltered) 

	g_filtered_Roll              = MSSA::lowPass2(g_filtered_Roll,              in.FieldRoll,              1.0f);
	g_filtered_Pitch             = MSSA::lowPass2(g_filtered_Pitch,             in.FieldPitch,             1.0f);
	g_filtered_Yaw               = MSSA::lowPass2(g_filtered_Yaw,               in.FieldYaw,               1.0f);
	g_filtered_RollSpeed         = MSSA::lowPass2(g_filtered_RollSpeed,         in.FieldRollSpeed,         1.0f);
	g_filtered_PitchSpeed        = MSSA::lowPass2(g_filtered_PitchSpeed,        in.FieldPitchSpeed,        1.0f);
	g_filtered_YawSpeed          = MSSA::lowPass2(g_filtered_YawSpeed,          in.FieldYawSpeed,          1.0f);
	g_filtered_RollAcceleration  = MSSA::lowPass2(g_filtered_RollAcceleration,  in.FieldRollAcceleration,  1.0f);
	g_filtered_PitchAcceleration = MSSA::lowPass2(g_filtered_PitchAcceleration, in.FieldPitchAcceleration, 1.0f);
	g_filtered_YawAcceleration   = MSSA::lowPass2(g_filtered_YawAcceleration,   in.FieldYawAcceleration,   1.0f);
	g_filtered_SurgeSpeed        = MSSA::lowPass2(g_filtered_SurgeSpeed,        in.FieldSurgeSpeed,        1.0f);
	g_filtered_HeaveSpeed        = MSSA::lowPass2(g_filtered_HeaveSpeed,        in.FieldHeaveSpeed,        1.0f);
	g_filtered_SwaySpeed         = MSSA::lowPass2(g_filtered_SwaySpeed,         in.FieldSwaySpeed,         1.0f);
	g_filtered_SurgeAcceleration = MSSA::lowPass2(g_filtered_SurgeAcceleration, in.FieldSurgeAcceleration, 1.0f);
	g_filtered_HeaveAcceleration = MSSA::lowPass2(g_filtered_HeaveAcceleration, in.FieldHeaveAcceleration, 1.0f);
	g_filtered_SwayAcceleration  = MSSA::lowPass2(g_filtered_SwayAcceleration,  in.FieldSwayAcceleration,  1.0f);

	// init control values 

	ctx.output.controlMode                         = MSSA::ControlMode_InverseKinematicsVector;
	ctx.output.motion.vectorMode.bestMatchStrategy = true;
	ctx.output.motion.vectorMode.roll_rad          = 0.0f;
	ctx.output.motion.vectorMode.pitch_rad         = 0.0f;
	ctx.output.motion.vectorMode.yaw_rad           = 0.0f;
	ctx.output.motion.vectorMode.heave_mm          = 0.0f;
	ctx.output.motion.vectorMode.sway_mm           = 0.0f;
 	ctx.output.motion.vectorMode.surge_mm          = 0.0f;

	// add kinematics components through third equ. 
	// Δx = ( v0 *​ t ) + ( 0.5​ * a * t^2 )

	ctx.output.motion.vectorMode.yaw_rad
		+= MSSA::bound(
			-ctx.singleExcursionsWorkEnvelope.yaw_rad,
			(
				g_previous_Yaw
				+ ( 
					( g_previous_YawSpeed * ( ctx.systemTimeDiff / 1000.0f ) ) 
					+ ( 0.5f * g_filtered_YawAcceleration * ( ( ctx.systemTimeDiff / 1000.0f ) * ( ctx.systemTimeDiff / 1000.0f ) ) )
				)
			),
			ctx.singleExcursionsWorkEnvelope.yaw_rad
		);

	ctx.output.motion.vectorMode.pitch_rad
		+= MSSA::bound(
			-ctx.singleExcursionsWorkEnvelope.pitch_rad,
			(
				g_previous_Pitch
				+ ( 
					( g_previous_PitchSpeed * ( ctx.systemTimeDiff / 1000.0f ) ) 
					+ ( 0.5f * g_filtered_PitchAcceleration * ( ( ctx.systemTimeDiff / 1000.0f ) * ( ctx.systemTimeDiff / 1000.0f ) ) )
				)
			),
			ctx.singleExcursionsWorkEnvelope.pitch_rad
		);

	ctx.output.motion.vectorMode.roll_rad
		+= MSSA::bound(
			-ctx.singleExcursionsWorkEnvelope.roll_rad,
			(
				g_previous_Roll
				+ ( 
					( g_previous_RollSpeed * ( ctx.systemTimeDiff / 1000.0f ) ) 
					+ ( 0.5f * g_filtered_RollAcceleration * ( ( ctx.systemTimeDiff / 1000.0f ) * ( ctx.systemTimeDiff / 1000.0f ) ) )
				)
			),
			ctx.singleExcursionsWorkEnvelope.roll_rad
		);
		
	ctx.output.motion.vectorMode.heave_mm
		+= MSSA::bound(
			-ctx.singleExcursionsWorkEnvelope.heave_mm,
			(
				g_previous_Heave
				+ ( 
					( g_previous_HeaveSpeed * ( ctx.systemTimeDiff / 1000.0f ) ) 
					+ ( 0.5f * g_filtered_HeaveAcceleration * ( ( ctx.systemTimeDiff / 1000.0f ) * ( ctx.systemTimeDiff / 1000.0f ) ) )
				)
			)
			/* meter (ISU input) to mm */ 
			* 1000.0f,
			ctx.singleExcursionsWorkEnvelope.heave_mm
		);

	ctx.output.motion.vectorMode.surge_mm
		+= MSSA::bound(
			-ctx.singleExcursionsWorkEnvelope.surge_mm,
			(
				g_previous_Surge
				+ ( 
					( g_previous_SurgeSpeed * ( ctx.systemTimeDiff / 1000.0f ) ) 
					+ ( 0.5f * g_filtered_SurgeAcceleration * ( ( ctx.systemTimeDiff / 1000.0f ) * ( ctx.systemTimeDiff / 1000.0f ) ) )
				)
			)
			/* meter (ISU input) to mm */ 
			* 1000.0f,
			ctx.singleExcursionsWorkEnvelope.surge_mm
		);

	ctx.output.motion.vectorMode.sway_mm
		+= MSSA::bound(
			-ctx.singleExcursionsWorkEnvelope.sway_mm,
			(
				g_previous_Sway
				+ ( 
					( g_previous_SwaySpeed * ( ctx.systemTimeDiff / 1000.0f ) ) 
					+ ( 0.5f * g_filtered_SwayAcceleration * ( ( ctx.systemTimeDiff / 1000.0f ) * ( ctx.systemTimeDiff / 1000.0f ) ) )
				)
			)
			/* meter (ISU input) to mm */ 
			* 1000.0f,
			ctx.singleExcursionsWorkEnvelope.sway_mm
		);

	// add unfiltered extra values

	ctx.output.motion.vectorMode.roll_rad  += in.FieldExtraRoll;
	ctx.output.motion.vectorMode.pitch_rad += in.FieldExtraPitch;
	ctx.output.motion.vectorMode.yaw_rad   += in.FieldExtraYaw;
	ctx.output.motion.vectorMode.sway_mm   += in.FieldExtraSway;
	ctx.output.motion.vectorMode.heave_mm  += in.FieldExtraHeave;
	ctx.output.motion.vectorMode.surge_mm  += in.FieldExtraSurge;

	// keep track of previous values

	g_previous_Roll       = ctx.output.motion.vectorMode.roll_rad;
	g_previous_Pitch      = ctx.output.motion.vectorMode.pitch_rad;
	g_previous_Yaw        = ctx.output.motion.vectorMode.yaw_rad;
	g_previous_Surge      = ctx.output.motion.vectorMode.sway_mm;
	g_previous_Heave      = ctx.output.motion.vectorMode.heave_mm;
	g_previous_Sway       = ctx.output.motion.vectorMode.surge_mm;
	g_previous_RollSpeed  = g_filtered_RollSpeed;
	g_previous_PitchSpeed = g_filtered_PitchSpeed;
	g_previous_YawSpeed   = g_filtered_YawSpeed;
	g_previous_SurgeSpeed = g_filtered_SurgeSpeed;
	g_previous_HeaveSpeed = g_filtered_HeaveSpeed;
	g_previous_SwaySpeed  = g_filtered_SwaySpeed;

	// other processors (defaulted)
	MoSy_FSMI_DefaultTactileAudioBasedFeedbackEffectsProcessor();
	MoSy_FSMI_DefaultSFXProcessor();

} /* process() */
