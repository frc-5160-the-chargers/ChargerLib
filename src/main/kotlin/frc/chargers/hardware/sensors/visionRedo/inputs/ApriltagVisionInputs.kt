package frc.chargers.hardware.sensors.visionRedo.inputs

import com.batterystaple.kmeasure.quantities.inUnit
import com.batterystaple.kmeasure.quantities.ofUnit
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.radians
import com.batterystaple.kmeasure.units.seconds
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import frc.chargers.hardware.sensors.visionRedo.VisionData
import frc.chargers.hardware.sensors.visionRedo.VisionResult
import frc.chargers.wpilibextensions.geometry.rotation.xAngle
import frc.chargers.wpilibextensions.geometry.rotation.yAngle
import frc.chargers.wpilibextensions.geometry.rotation.zAngle
import frc.chargers.wpilibextensions.geometry.threedimensional.UnitTransform3d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

public class ApriltagVisionInputs: LoggableInputs {


    public var data: VisionData<VisionResult.Apriltag>? = null

    override fun toLog(table: LogTable?) {
        table?.apply{
            put("VisionDataValid", data != null)
            data?.let{
                put("timestampSecs", it.timestamp.inUnit(seconds))

                put("BestTarget/tx", it.bestTarget.tx)
                put("BestTarget/ty", it.bestTarget.ty)
                put("BestTarget/areaPercent", it.bestTarget.areaPercent)
                put("BestTarget/id", it.bestTarget.id.toLong())
                with(it.bestTarget.targetTransformFromCam){
                    put("BestTarget/TransformFromCamera/x",x.inUnit(meters))
                    put("BestTarget/TransformFromCamera/y", y.inUnit(meters))
                    put("BestTarget/TransformFromCamera/z", z.inUnit(meters))
                    put("BestTarget/TransformFromCamera/yaw", rotation.xAngle.inUnit(radians))
                    put("BestTarget/TransformFromCamera/pitch", rotation.yAngle.inUnit(radians))
                    put("BestTarget/TransformFromCamera/roll", rotation.zAngle.inUnit(radians))
                }

                put("OtherTargets/tx", it.otherTargets.map{ target -> target.tx}.toDoubleArray())
                put("OtherTargets/ty", it.otherTargets.map{ target -> target.ty}.toDoubleArray())
                put("OtherTargets/areaPercent", it.otherTargets.map{ target -> target.areaPercent}.toDoubleArray())
                put("OtherTargets/id", it.otherTargets.map{ target -> target.id.toLong()}.toLongArray())

                put("OtherTargets/TransformFromCamera/x",it.otherTargets.map{target -> target.targetTransformFromCam.x.inUnit(
                    meters
                )}.toDoubleArray())
                put("OtherTargets/TransformFromCamera/y", it.otherTargets.map{target -> target.targetTransformFromCam.y.inUnit(
                    meters
                )}.toDoubleArray())
                put("OtherTargets/TransformFromCamera/z", it.otherTargets.map{target -> target.targetTransformFromCam.z.inUnit(
                    meters
                )}.toDoubleArray())
                put("OtherTargets/TransformFromCamera/yaw", it.otherTargets.map{target -> target.targetTransformFromCam.rotation.xAngle.inUnit(
                    radians
                )}.toDoubleArray())
                put("OtherTargets/TransformFromCamera/pitch", it.otherTargets.map{target -> target.targetTransformFromCam.rotation.yAngle.inUnit(
                    radians
                )}.toDoubleArray())
                put("OtherTargets/TransformFromCamera/roll", it.otherTargets.map{target -> target.targetTransformFromCam.rotation.zAngle.inUnit(
                    radians
                )}.toDoubleArray())
            }
        }

    }

    override fun fromLog(table: LogTable?) {
        table?.apply{
            val bestTargetInput = VisionResult.Apriltag(
                getDouble("BestTarget/tx",0.0),
                getDouble("BestTarget/ty",0.0),
                getDouble("BestTarget/areaPercent",0.0),
                getInteger("BestTarget/id",0).toInt(),
                UnitTransform3d(
                    Transform3d(
                        Translation3d(
                            getDouble("BestTarget/TransformFromCamera/x",0.0),
                            getDouble("BestTarget/TransformFromCamera/y",0.0),
                            getDouble("BestTarget/TransformFromCamera/z",0.0),
                        ),
                        Rotation3d(
                            getDouble("BestTarget/TransformFromCamera/roll",0.0),
                            getDouble("BestTarget/TransformFromCamera/pitch",0.0),
                            getDouble("BestTarget/TransformFromCamera/yaw",0.0),
                        )
                    )
                )
            )

            val allTXValues = getDoubleArray("OtherTargets/tx", doubleArrayOf())
            val allTYValues = getDoubleArray("OtherTargets/ty", doubleArrayOf())
            val allAreaValues = getDoubleArray("OtherTargets/areaPercent", doubleArrayOf())
            val allIDs = getDoubleArray("OtherTargets/id", doubleArrayOf()).map{it.toInt()}

            val allTransformXValues = getDoubleArray("OtherTargets/TransformFromCamera/x", doubleArrayOf())
            val allTransformYValues = getDoubleArray("OtherTargets/TransformFromCamera/y", doubleArrayOf())
            val allTransformZValues = getDoubleArray("OtherTargets/TransformFromCamera/z", doubleArrayOf())
            val allTransformRollValues = getDoubleArray("OtherTargets/TransformFromCamera/roll", doubleArrayOf())
            val allTransformPitchValues = getDoubleArray("OtherTargets/TransformFromCamera/pitch", doubleArrayOf())
            val allTransformYawValues = getDoubleArray("OtherTargets/TransformFromCamera/yaw", doubleArrayOf())


            require(allTXValues.size == allTYValues.size && allAreaValues.size == allIDs.size){"The sizes of the array values logged do not match."}

            val otherTags: MutableList<VisionResult.Apriltag> = mutableListOf()
            for (i in allTXValues.indices){
                otherTags.add(
                    VisionResult.Apriltag(
                        allTXValues[i],
                        allTYValues[i],
                        allAreaValues[i],
                        allIDs[i],
                        UnitTransform3d(
                            Transform3d(
                                Translation3d(
                                    allTransformXValues[i],
                                    allTransformYValues[i],
                                    allTransformZValues[i]
                                ),
                                Rotation3d(
                                    allTransformRollValues[i],
                                    allTransformPitchValues[i],
                                    allTransformYawValues[i]
                                )
                            )
                        )
                    )

                )
            }

            data = VisionData(
                getDouble("timestampSecs",0.0).ofUnit(seconds),
                bestTargetInput,
                otherTags
            )

        }

    }

}