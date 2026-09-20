#!/usr/bin/env python3
"""Compare compact and full URDF models without starting ROS or Isaac."""
import argparse
import os
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from ammr_teleop.common.kinematics import DualArmKinematics
from ammr_teleop.common.robot_ik import ArmIK

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-root',type=Path,required=True)
    args=parser.parse_args()
    model_root=args.reference_root/'assets/robot/Agilex_model'
    os.environ['AMMR_URDF']=str(model_root/'c2436/urdf/robot_v2_chassis.urdf')
    os.environ['AMMR_URDF_PACKAGE_DIRS']=str(model_root)
    old=ArmIK()
    os.environ['AMMR_URDF']=str(Path(__file__).resolve().parents[1]/'models/kinematics/c2436.urdf')
    os.environ.pop('AMMR_URDF_PACKAGE_DIRS',None)
    new=DualArmKinematics('')
    a,b=old.reduced_robot.model,new.ik.reduced_robot.model
    assert list(a.names)==list(b.names)
    np.testing.assert_array_equal(a.lowerPositionLimit,b.lowerPositionLimit)
    np.testing.assert_array_equal(a.upperPositionLimit,b.upperPositionLimit)
    rng=np.random.default_rng(20260905)
    maximum=0.
    for _ in range(40):
        q=rng.uniform(a.lowerPositionLimit,a.upperPositionLimit)
        expected=old.reduced_robot.model.createData()
        new.pin.forwardKinematics(a,expected,q)
        new.pin.updateFramePlacements(a,expected)
        actual=new._forward_reduced_q(q)
        for frame,pose in zip((old.L_hand_id,old.R_hand_id),actual):
            maximum=max(maximum,float(np.max(np.abs(expected.oMf[frame].homogeneous-pose))))
            np.testing.assert_allclose(expected.oMf[frame].homogeneous,pose,atol=1e-12,rtol=0)
    print(f'MODEL_EQUIVALENT samples=40 nq={b.nq} max_FK_error={maximum:.3g}')

if __name__=='__main__':main()
