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
	float g_previous_Time              = 0.0;

} /* } anonymous */

MOSY_SCRIPT_API_EXPORT void setup(MSSA::Context& ctx, MSSA::System& /*sys*/)
{

	g_previous_Time = ctx.input.fromGame.named.FieldTime;

}

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

	// security bound 

	const auto LeftRightMaxAngle  = ctx.singleExcursionsWorkEnvelope.roll_rad;
	const auto RearFrontMaxAngle  = ctx.singleExcursionsWorkEnvelope.pitch_rad;
	const auto YawMaxAngle        = ctx.singleExcursionsWorkEnvelope.yaw_rad;
	const auto DownUpMaxOffset    = ctx.singleExcursionsWorkEnvelope.heave_mm;
	const auto LeftRightMaxOffset = ctx.singleExcursionsWorkEnvelope.sway_mm;
	const auto RearFrontMaxOffset = ctx.singleExcursionsWorkEnvelope.surge_mm;

	// add kinematics components

	if (g_filtered_RollAcceleration != 0.0f) 
	{

		// fourth kinematics equation

		ctx.output.motion.vectorMode.roll_rad
			+= MSSA::bound(
				-LeftRightMaxAngle,
				g_previous_Roll + ((g_filtered_RollSpeed * g_filtered_RollSpeed) - (g_previous_RollSpeed * g_previous_RollSpeed)) / (2.0f * g_filtered_RollAcceleration),
				LeftRightMaxAngle
			);
		ctx.output.motion.vectorMode.pitch_rad 
			+= MSSA::bound(
				-RearFrontMaxAngle,
				g_previous_Pitch + ((g_filtered_PitchSpeed * g_filtered_PitchSpeed) - (g_previous_PitchSpeed * g_previous_PitchSpeed)) / (2.0f * g_filtered_PitchAcceleration),
				RearFrontMaxAngle
			);
		ctx.output.motion.vectorMode.yaw_rad   
			+= MSSA::bound(
				-YawMaxAngle,
				g_previous_Yaw + ((g_filtered_YawSpeed * g_filtered_YawSpeed) - (g_previous_YawSpeed * g_previous_YawSpeed)) / (2.0f * g_filtered_YawAcceleration),
				YawMaxAngle
			);
		ctx.output.motion.vectorMode.heave_mm  
			+= MSSA::bound(
				-DownUpMaxOffset,
				g_previous_Heave + ((g_filtered_HeaveSpeed * g_filtered_HeaveSpeed) - (g_previous_HeaveSpeed * g_previous_HeaveSpeed)) / (2.0f * g_filtered_HeaveAcceleration),
				DownUpMaxOffset
			);
		ctx.output.motion.vectorMode.sway_mm  
			+= MSSA::bound(
				-LeftRightMaxOffset,
				g_previous_Sway + ((g_filtered_SwaySpeed * g_filtered_SwaySpeed) - (g_previous_SwaySpeed * g_previous_SwaySpeed)) / (2.0f * g_filtered_SwayAcceleration),
				LeftRightMaxOffset
			);
		ctx.output.motion.vectorMode.surge_mm 
			+= MSSA::bound(
				-RearFrontMaxOffset,
				g_previous_Surge + ((g_filtered_SurgeSpeed * g_filtered_SurgeSpeed) - (g_previous_SurgeSpeed * g_previous_SurgeSpeed)) / (2.0f * g_filtered_SurgeAcceleration),
				RearFrontMaxOffset
			);
	}
	else
	{

		// second kinematics equation

		ctx.output.motion.vectorMode.roll_rad
			+= MSSA::bound(
				-LeftRightMaxAngle,
				g_previous_Roll + (((g_filtered_RollSpeed + g_previous_RollSpeed) / 2.0f) * ((in.FieldTime - g_previous_Time) / 1000.0f)),
				LeftRightMaxAngle
			);
		ctx.output.motion.vectorMode.pitch_rad 
			+= MSSA::bound(
				-RearFrontMaxAngle,
				g_previous_Pitch + (((g_filtered_PitchSpeed + g_previous_PitchSpeed) / 2.0f) * ((in.FieldTime - g_previous_Time) / 1000.0f)),
				RearFrontMaxAngle
			);
		ctx.output.motion.vectorMode.yaw_rad   
			+= MSSA::bound(
				-YawMaxAngle,
				g_previous_Yaw + (((g_filtered_YawSpeed + g_previous_YawSpeed) / 2.0f) * ((in.FieldTime - g_previous_Time) / 1000.0f)),
				YawMaxAngle
			);
		ctx.output.motion.vectorMode.heave_mm  
			+= MSSA::bound(
				-DownUpMaxOffset,
				g_previous_Heave + (((g_filtered_HeaveSpeed + g_previous_HeaveSpeed) / 2.0f) * ((in.FieldTime - g_previous_Time) / 1000.0f)),
				DownUpMaxOffset
			);
		ctx.output.motion.vectorMode.sway_mm  
			+= MSSA::bound(
				-LeftRightMaxOffset,
				g_previous_Sway + (((g_filtered_SwaySpeed + g_previous_SwaySpeed) / 2.0f) * ((in.FieldTime - g_previous_Time) / 1000.0f)),
				LeftRightMaxOffset
			);
		ctx.output.motion.vectorMode.surge_mm 
			+= MSSA::bound(
				-RearFrontMaxOffset,
				g_previous_Surge + (((g_filtered_SurgeSpeed + g_previous_SurgeSpeed) / 2.0f) * ((in.FieldTime - g_previous_Time) / 1000.0f)),
				RearFrontMaxOffset
			);

	} /* if() */

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
	g_previous_Time       = in.FieldTime;

	// other processors (defaulted)
	MoSy_FSMI_DefaultTactileAudioBasedFeedbackEffectsProcessor();
	MoSy_FSMI_DefaultSFXProcessor();

} /* process() */
