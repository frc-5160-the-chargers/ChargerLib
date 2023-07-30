package frc.chargers.wpilibextensions.geometry


import com.batterystaple.kmeasure.quantities.Distance
import com.batterystaple.kmeasure.quantities.ofUnit
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d



public fun Pose2d.ofUnit(unit: Distance): UnitPose2d = UnitPose2d(this.x.ofUnit(unit),this.y.ofUnit(unit),this.rotation.asAngle())
public fun Translation2d.ofUnit(unit: Distance): UnitTranslation2d = UnitTranslation2d(this.norm.ofUnit(unit),this.angle.asAngle())
public fun Transform2d.ofUnit(unit: Distance): UnitTransform2d = UnitTransform2d(this.translation.ofUnit(unit),this.rotation.asAngle())
