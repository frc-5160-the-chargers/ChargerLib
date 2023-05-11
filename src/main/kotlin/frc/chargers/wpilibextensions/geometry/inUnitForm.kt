package frc.chargers.wpilibextensions.geometry

import com.batterystaple.kmeasure.quantities.Angle
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d

public fun Pose2d.inUnitForm(): UnitPose2d = UnitPose2d(this.x.meters,this.y.meters,this.rotation.inUnitForm())
public fun Rotation2d.inUnitForm(): UnitRotation2d = UnitRotation2d(this.degrees.degrees)
public fun Translation2d.inUnitForm(): UnitTranslation2d = UnitTranslation2d(this.norm.meters,this.angle.inUnitForm())
public fun Transform2d.inUnitForm(): UnitTransform2d = UnitTransform2d(this.translation.inUnitForm(),this.rotation.inUnitForm())








