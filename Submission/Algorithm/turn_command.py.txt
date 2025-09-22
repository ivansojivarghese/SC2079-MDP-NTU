import math
from misc.type_of_turn import TypeOfTurn
from misc.direction import Direction
from misc.positioning import Position, RobotPosition
from commands.command import Command
import constants as constants


class TurnCommand(Command):
    def __init__(self, type_of_turn, left, right, reverse):
        """
        Angle to turn and whether the turn is done in reverse or not. Note that this is in degrees.

        Note that negative angles will always result in the robot being rotated clockwise.
        """
        time = 0

        if type_of_turn == TypeOfTurn.MEDIUM:
            time = 20  # SOME VALUE TO BE EMPIRICALLY DETERMINED
    
        super().__init__(time)
        self.type_of_turn = type_of_turn
        self.left = left
        self.right = right
        self.reverse = reverse

    def __str__(self):
        # return f"TurnCommand:{self.type_of_turn}, {self.total_ticks} ticks, rev={self.reverse}, left={self.left}, right={self.right}) "
        return f"TurnCommand:{self.type_of_turn}, rev={self.reverse}, left={self.left}, right={self.right}) "

    __repr__ = __str__

    def process_one_tick(self, robot):
        if self.total_ticks == 0:
            return

        self.tick()
        robot.turn(self.type_of_turn, self.left, self.right, self.reverse)

    def get_type_of_turn(self):
        return self.type_of_turn

    def apply_on_pos(self, curr_pos: Position):
        """
        changes the robot position according to what command it is and where the robot is currently at
        """
        assert isinstance(curr_pos, RobotPosition), print(
            "Cannot apply turn command on non-robot positions!")

        # Get change in (x, y) coordinate.

        if self.left and not self.right and not self.reverse:

            if curr_pos.direction == Direction.TOP:
                curr_pos.x += constants.TURN_MED_FORWARD_LEFT_TOP[0]   # (-30)
                curr_pos.y += constants.TURN_MED_FORWARD_LEFT_TOP[1]   # (20)
                curr_pos.direction = Direction.LEFT
            elif curr_pos.direction == Direction.LEFT:
                curr_pos.x += constants.TURN_MED_FORWARD_LEFT_LEFT[0]  # (-20)
                curr_pos.y += constants.TURN_MED_FORWARD_LEFT_LEFT[1]  # (-30)
                curr_pos.direction = Direction.BOTTOM
            elif curr_pos.direction == Direction.RIGHT:
                curr_pos.x += constants.TURN_MED_FORWARD_LEFT_RIGHT[0] # (20)
                curr_pos.y += constants.TURN_MED_FORWARD_LEFT_RIGHT[1] # (30)
                curr_pos.direction = Direction.TOP
            elif curr_pos.direction == Direction.BOTTOM:
                curr_pos.x += constants.TURN_MED_FORWARD_LEFT_BOTTOM[0]# (30)
                curr_pos.y += constants.TURN_MED_FORWARD_LEFT_BOTTOM[1]# (-20)
                curr_pos.direction = Direction.RIGHT

        # turn right and forward
        if self.right and not self.left and not self.reverse:
            
            if curr_pos.direction == Direction.TOP:
                curr_pos.x += constants.TURN_MED_FORWARD_RIGHT_TOP[0]   # (30) 
                curr_pos.y += constants.TURN_MED_FORWARD_RIGHT_TOP[1]   # (20) – note: if you want to change 20 to 70, update the constant
                curr_pos.direction = Direction.RIGHT
            elif curr_pos.direction == Direction.LEFT:
                curr_pos.x += constants.TURN_MED_FORWARD_RIGHT_LEFT[0]  # (-20)
                curr_pos.y += constants.TURN_MED_FORWARD_RIGHT_LEFT[1]  # (30)
                curr_pos.direction = Direction.TOP
            elif curr_pos.direction == Direction.RIGHT:
                curr_pos.x += constants.TURN_MED_FORWARD_RIGHT_RIGHT[0] # (20)
                curr_pos.y += constants.TURN_MED_FORWARD_RIGHT_RIGHT[1] # (-30)
                curr_pos.direction = Direction.BOTTOM
            elif curr_pos.direction == Direction.BOTTOM:
                curr_pos.x += constants.TURN_MED_FORWARD_RIGHT_BOTTOM[0]# (-30)
                curr_pos.y += constants.TURN_MED_FORWARD_RIGHT_BOTTOM[1]# (-20)
                curr_pos.direction = Direction.LEFT

        # turn front wheels left and reverse
        if self.left and not self.right and self.reverse:

            if curr_pos.direction == Direction.TOP:
                curr_pos.x += constants.TURN_MED_REVERSE_LEFT_TOP[0]   # (-20)
                curr_pos.y += constants.TURN_MED_REVERSE_LEFT_TOP[1]   # (-30)
                curr_pos.direction = Direction.RIGHT
            elif curr_pos.direction == Direction.LEFT:
                curr_pos.x += constants.TURN_MED_REVERSE_LEFT_LEFT[0]  # (30)
                curr_pos.y += constants.TURN_MED_REVERSE_LEFT_LEFT[1]  # (-20)
                curr_pos.direction = Direction.TOP
            elif curr_pos.direction == Direction.RIGHT:
                curr_pos.x += constants.TURN_MED_REVERSE_LEFT_RIGHT[0] # (-30)
                curr_pos.y += constants.TURN_MED_REVERSE_LEFT_RIGHT[1] # (20)
                curr_pos.direction = Direction.BOTTOM
            elif curr_pos.direction == Direction.BOTTOM:
                curr_pos.x += constants.TURN_MED_REVERSE_LEFT_BOTTOM[0]# (20)
                curr_pos.y += constants.TURN_MED_REVERSE_LEFT_BOTTOM[1]# (30)
                curr_pos.direction = Direction.LEFT

        # turn front wheels right and reverse
        if self.right and not self.left and self.reverse:

            if curr_pos.direction == Direction.TOP:
                curr_pos.x += constants.TURN_MED_REVERSE_RIGHT_TOP[0]   # (20)
                curr_pos.y += constants.TURN_MED_REVERSE_RIGHT_TOP[1]   # (-30)
                curr_pos.direction = Direction.LEFT
            elif curr_pos.direction == Direction.LEFT:
                curr_pos.x += constants.TURN_MED_REVERSE_RIGHT_LEFT[0]  # (30)
                curr_pos.y += constants.TURN_MED_REVERSE_RIGHT_LEFT[1]  # (20)
                curr_pos.direction = Direction.BOTTOM
            elif curr_pos.direction == Direction.RIGHT:
                curr_pos.x += constants.TURN_MED_REVERSE_RIGHT_RIGHT[0] # (-30)
                curr_pos.y += constants.TURN_MED_REVERSE_RIGHT_RIGHT[1] # (-20)
                curr_pos.direction = Direction.TOP
            elif curr_pos.direction == Direction.BOTTOM:
                curr_pos.x += constants.TURN_MED_REVERSE_RIGHT_BOTTOM[0]# (-20)
                curr_pos.y += constants.TURN_MED_REVERSE_RIGHT_BOTTOM[1]# (30)
                curr_pos.direction = Direction.RIGHT

        return self

    def convert_to_message(self):
        indoor = False
        # indoor
        if indoor:
            if self.left and not self.right and not self.reverse:
                return "FL000"  # turn left medium forward!
            elif self.left and not self.right and self.reverse:
                return "BL000"  # turn left medium reverse!

            elif self.right and not self.left and not self.reverse:
                return "FR000"  # turn right medium forward!
            else:
                # This is going backward and the front wheels turned to the right.
                return "BR000"  # turn right medium reverse!
        else:
            # outdoor
            if self.left and not self.right and not self.reverse:
                return "FL030"  # turn left medium forward!
            elif self.left and not self.right and self.reverse:
                return "BL030"  # turn left medium reverse!

            elif self.right and not self.left and not self.reverse:
                return "FR030"  # turn right medium forward!
            else:
                # This is going backward and the front wheels turned to the right.
                return "BR030"  # turn right medium reverse!

