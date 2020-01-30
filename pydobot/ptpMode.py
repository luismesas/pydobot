from enum import Enum


# ptpMode: int // PTP mode (Value range 0-9)
# x, y, z, r: float
# x,y,z,r are the parameters of ptpMode, which can be set as coordinates, joint angles, or angle increments

class PtPMode(Enum):
    """
    0. JUMP_XYZ, (x,y,z,r) is the target point in Cartesian coordinate system
    1. MOVJ_XYZ, (x,y,z,r) is the target point in Cartesian coordinate system
    2. MOVL_XYZ, (x,y,z,r) is the target point in Cartesian coordinate system
    3. JUMP_ANGLE, (x,y,z,r) is the target point in Jointcoordinate system
    4. MOVJ_ANGLE, joint movement, (x,y,z,r) is the target point in Joint coordinate system
    5. MOVL_ANGLE, linear movement, (x,y,z,r) is the target point in Joint coordinate system
    6. MOVJ_INC, joint movement increment mode, (x,y,z,r) is the angle increment in Joint coordinate system
    7. MOVL_INC, linear movement increment mode, (x,y,z,r) is the Cartesian coordinate increment in Joint coordinate system
    8. MOVJ_XYZ_INC, joint movement increment mode, (x,y,z,r) is the Cartesian coordinate increment in Cartesian coordinate system
    9. JUMP_MOVL_XYZ, (x,y,z,r) is the Cartesian coordinate increment in Cartesian coordinate system
    """
    JUMP_XYZ = 0x00
    MOVJ_XYZ = 0x01
    MOVL_XYZ = 0x02
    JUMP_ANGLE = 0x03
    MOVJ_ANGLE = 0x04
    MOVL_ANGLE = 0x05
    MOVJ_INC = 0x06
    MOVL_INC = 0x07
    MOVJ_XYZ_INC = 0x08
    JUMP_MOVL_XYZ = 0x09
