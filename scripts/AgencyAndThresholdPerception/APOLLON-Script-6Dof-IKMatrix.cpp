#include "GameSpecific.h"
#include "ScriptsAPI.h"

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

	// control values 
	ctx.output.controlMode           = MSSA::ControlMode_InverseKinematicsMatrix;
	ctx.output.motion.matrixMode.M11 = in.FieldMatrix11;
	ctx.output.motion.matrixMode.M12 = in.FieldMatrix12;
	ctx.output.motion.matrixMode.M13 = in.FieldMatrix13;
	ctx.output.motion.matrixMode.M14 = in.FieldMatrix14;
	ctx.output.motion.matrixMode.M21 = in.FieldMatrix21;
	ctx.output.motion.matrixMode.M22 = in.FieldMatrix22;
	ctx.output.motion.matrixMode.M23 = in.FieldMatrix23;
	ctx.output.motion.matrixMode.M24 = in.FieldMatrix24;
	ctx.output.motion.matrixMode.M31 = in.FieldMatrix31;
	ctx.output.motion.matrixMode.M32 = in.FieldMatrix32;
	ctx.output.motion.matrixMode.M33 = in.FieldMatrix33;
	ctx.output.motion.matrixMode.M34 = in.FieldMatrix34;
	ctx.output.motion.matrixMode.M41 = in.FieldMatrix41;
	ctx.output.motion.matrixMode.M42 = in.FieldMatrix42;
	ctx.output.motion.matrixMode.M43 = in.FieldMatrix43;
	ctx.output.motion.matrixMode.M44 = in.FieldMatrix44;

	// other processors (defaulted)
	MoSy_FSMI_DefaultTactileAudioBasedFeedbackEffectsProcessor();
	MoSy_FSMI_DefaultSFXProcessor();

} /* process() */
