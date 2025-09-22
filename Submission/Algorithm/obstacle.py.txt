import pygame
import constants
from misc.direction import Direction
from misc.positioning import Position, RobotPosition


class Obstacle:
    def __init__(self, position: Position, index):
        """
        x -> x-coordinate of the obstacle.
        y -> y-coordinate of the obstacle.
        direction -> Which direction the image is facing. If image is on the right side of the obstacle, RIGHT.
        """
        # Check if the coordinates are multiples of 10 with offset 5. If they are not, then they are invalid
        # obstacle coordinates.
        # This is from the assumption that all obstacles are placed centered in each grid.
        if position.x % 10 != 0 or position.y % 10 != 0:
            raise AssertionError(
                "Obstacle center coordinates must be multiples of 10!")

        self.position = position
        self.scale = 2
        # Target position for permutation
        self.target_position = self.get_robot_target_pos()

        # Arrow to draw at the target coordinate.
        # self.target_image = pygame.transform.scale(pygame.image.load("entities/effects/target-pointer.png"),
        #                                            (25, 25))

        self.index = index

    def __str__(self):
        return f"Obstacle({self.position})"

    __repr__ = __str__

    def check_within_boundary(self, position, yolo):
        """
        Checks whether a given position is within the safety boundary of this obstacle.
        If True, the robot is too close and may hit the obstacle.
        However, with higher retry levels (yolo), we shrink the effective safety boundary,
        allowing the robot to take a more dangerous path.
        
        Args:
            position: A Position object representing the robot's center.
            yolo: An integer representing the retry level; higher means more risk accepted.
            
        Returns:
            True if the position is within the (adjusted) safety boundary, False otherwise.
        """
        # Compute the effective safety boundary.
        # At yolo = 0, we use the full safety width (+1 for inclusive check).
        # For each increase in yolo, we reduce the margin by 2 units.
        # Ensure the effective margin never falls below a minimum (e.g., 1 unit).
        effective_safety = max(constants.OBSTACLE_SAFETY_WIDTH + 1 - yolo * 2, -1)

        # Determine the 3x3 grid positions around the robot.
        x_range = [position.x - constants.GRID_CELL_LENGTH, 
                position.x, 
                position.x + constants.GRID_CELL_LENGTH]
        y_range = [position.y - constants.GRID_CELL_LENGTH,
                position.y, 
                position.y + constants.GRID_CELL_LENGTH]

        # Check each point in the 3x3 grid to see if it lies within the (adjusted) safety boundary.
        for x in x_range:
            for y in y_range:
                # cross
                if yolo == 1 and not (position.x == x or position.y == y):
                    continue

                # 1x1
                if yolo == 2 and not (position.x == x and position.y == y):
                    continue
                diffX = abs(self.position.x - x)
                diffY = abs(self.position.y - y)
                if diffX < effective_safety and diffY < effective_safety:
                    return True

        return False


    def get_boundary_points(self):
        """
        Get points at the corner of the virtual obstacle for this image.

        Useful for checking if a point is within the boundary of this obstacle.
        """
        upper = self.position.y + constants.OBSTACLE_SAFETY_WIDTH
        lower = self.position.y - constants.OBSTACLE_SAFETY_WIDTH
        left = self.position.x - constants.OBSTACLE_SAFETY_WIDTH
        right = self.position.x + constants.OBSTACLE_SAFETY_WIDTH

        return [
            # Note that in this case, the direction does not matter.
            Position(left, lower),  # Bottom left.
            Position(right, lower),  # Bottom right.
            Position(left, upper),  # Upper left.
            Position(right, upper)  # Upper right.
        ]
    def get_robot_target_pos(self, offset=None):
        """
        Returns the point that the robot should target for, including the target orientation.
        The offset is applied in a direction-dependent manner:
        - If target face is BOTTOM (facing down), subtract offset from y.
        - If target face is TOP (facing up), add offset to y.
        - If target face is LEFT (facing left), subtract offset from x.
        - If target face is RIGHT (facing right), add offset to x.
        
        By default, offset is 10 (meaning a 10cm adjustment from the base calculation).
        """
        if offset is None:
            offset = 10  # default offset (set to 0 for a "closer" scan)

        # For brevity, define some common values.
        safety = constants.OBSTACLE_SAFETY_WIDTH * self.scale
        base = constants.OBSTACLE_LENGTH  # base distance from the obstacle's face

        # Bottom left corner edge case
        if self.position.y == 0 and self.position.x == 0:
            if self.position.direction == Direction.TOP:
                # Target face: BOTTOM → subtract offset from y.
                return RobotPosition(
                    self.position.x + 10,
                    self.position.y + safety + base - offset,
                    Direction.BOTTOM
                )
            elif self.position.direction == Direction.BOTTOM:
                # Target face: TOP → add offset to y.
                return RobotPosition(
                    self.position.x,
                    self.position.y - safety - base + offset,
                    Direction.TOP
                )
            elif self.position.direction == Direction.LEFT:
                # Target face: RIGHT → add offset to x.
                return RobotPosition(
                    self.position.x - safety - base + offset,
                    self.position.y,
                    Direction.RIGHT
                )
            else:  # self.position.direction == Direction.RIGHT
                # Target face: LEFT → subtract offset from x.
                return RobotPosition(
                    self.position.x + safety + base - offset,
                    self.position.y + 10,
                    Direction.LEFT
                )

        # Top left corner edge case
        elif self.position.y == 190 and self.position.x == 0:
            if self.position.direction == Direction.TOP:
                # Target face: BOTTOM → subtract offset from y.
                return RobotPosition(
                    self.position.x,
                    self.position.y + safety + base - offset,
                    Direction.BOTTOM
                )
            elif self.position.direction == Direction.BOTTOM:
                # Target face: TOP → add offset to y.
                return RobotPosition(
                    self.position.x + 10,
                    self.position.y - safety - base + offset,
                    Direction.TOP
                )
            elif self.position.direction == Direction.LEFT:
                # Target face: RIGHT → add offset to x.
                return RobotPosition(
                    self.position.x - safety - base + offset,
                    self.position.y,
                    Direction.RIGHT
                )
            else:  # Direction.RIGHT
                # Target face: LEFT → subtract offset from x.
                return RobotPosition(
                    self.position.x + safety + base - offset,
                    self.position.y - 10,
                    Direction.LEFT
                )

        # Top right corner edge case
        elif self.position.y == 190 and self.position.x == 190:
            if self.position.direction == Direction.TOP:
                # Target face: BOTTOM → subtract offset from y.
                return RobotPosition(
                    self.position.x,
                    self.position.y + safety + base - offset,
                    Direction.BOTTOM
                )
            elif self.position.direction == Direction.BOTTOM:
                # Target face: TOP → add offset to y.
                return RobotPosition(
                    self.position.x - 10,
                    self.position.y - safety - base + offset,
                    Direction.TOP
                )
            elif self.position.direction == Direction.LEFT:
                # Target face: RIGHT → add offset to x.
                return RobotPosition(
                    self.position.x - safety - base + offset,
                    self.position.y - 10,
                    Direction.RIGHT
                )
            else:  # Direction.RIGHT
                # Target face: LEFT → subtract offset from x.
                return RobotPosition(
                    self.position.x + safety + base - offset,
                    self.position.y,
                    Direction.LEFT
                )

        # Bottom right corner edge case
        elif self.position.y == 0 and self.position.x == 190:
            if self.position.direction == Direction.TOP:
                # Target face: BOTTOM → subtract offset from y.
                return RobotPosition(
                    self.position.x - 10,
                    self.position.y + safety + base - offset,
                    Direction.BOTTOM
                )
            elif self.position.direction == Direction.BOTTOM:
                # Target face: TOP → add offset to y.
                return RobotPosition(
                    self.position.x,
                    self.position.y - safety - base + offset,
                    Direction.TOP
                )
            elif self.position.direction == Direction.LEFT:
                # Target face: RIGHT → add offset to x.
                return RobotPosition(
                    self.position.x - safety - base + offset,
                    self.position.y + 10,
                    Direction.RIGHT
                )
            else:  # Direction.RIGHT
                # Target face: LEFT → subtract offset from x.
                return RobotPosition(
                    self.position.x + safety + base - offset,
                    self.position.y,
                    Direction.LEFT
                )

        # Obstacles along the bottom (but not at the extreme left/right)
        elif self.position.y == 0:
            if self.position.direction == Direction.TOP:
                # Target face: BOTTOM → subtract offset from y.
                return RobotPosition(
                    self.position.x,
                    self.position.y + safety + base - offset,
                    Direction.BOTTOM
                )
            elif self.position.direction == Direction.BOTTOM:
                # Target face: TOP → add offset to y.
                return RobotPosition(
                    self.position.x,
                    self.position.y - safety - base + offset,
                    Direction.TOP
                )
            elif self.position.direction == Direction.LEFT:
                # Target face: RIGHT → add offset to x.
                return RobotPosition(
                    self.position.x - safety - base + offset,
                    self.position.y + 10,
                    Direction.RIGHT
                )
            else:  # Direction.RIGHT
                # Target face: LEFT → subtract offset from x.
                return RobotPosition(
                    self.position.x + safety + base - offset,
                    self.position.y + 10,
                    Direction.LEFT
                )

        # Obstacles along the top (but not at the extreme left/right)
        elif self.position.y == 190:
            if self.position.direction == Direction.TOP:
                # Target face: BOTTOM → subtract offset from y.
                return RobotPosition(
                    self.position.x,
                    self.position.y + safety + base - offset,
                    Direction.BOTTOM
                )
            elif self.position.direction == Direction.BOTTOM:
                # Target face: TOP → add offset to y.
                return RobotPosition(
                    self.position.x,
                    self.position.y - safety - base + offset,
                    Direction.TOP
                )
            elif self.position.direction == Direction.LEFT:
                # Target face: RIGHT → add offset to x.
                return RobotPosition(
                    self.position.x - safety - base + offset,
                    self.position.y - 10,
                    Direction.RIGHT
                )
            else:  # Direction.RIGHT
                # Target face: LEFT → subtract offset from x.
                return RobotPosition(
                    self.position.x + safety + base - offset,
                    self.position.y - 10,
                    Direction.LEFT
                )

        # Obstacles along the left side (but not at the extreme top/bottom)
        elif self.position.x == 0:
            if self.position.direction == Direction.TOP:
                # Target face: BOTTOM → subtract offset from y.
                return RobotPosition(
                    self.position.x + 10,
                    self.position.y + safety + base - offset,
                    Direction.BOTTOM
                )
            elif self.position.direction == Direction.BOTTOM:
                # Target face: TOP → add offset to y.
                return RobotPosition(
                    self.position.x + 10,
                    self.position.y - safety - base + offset,
                    Direction.TOP
                )
            elif self.position.direction == Direction.LEFT:
                # Target face: RIGHT → add offset to x.
                return RobotPosition(
                    self.position.x - safety - base + offset,
                    self.position.y + 10,
                    Direction.RIGHT
                )
            else:  # Direction.RIGHT
                # Target face: LEFT → subtract offset from x.
                return RobotPosition(
                    self.position.x + safety + base - offset,
                    self.position.y,
                    Direction.LEFT
                )

        # Obstacles along the right side (but not at the extreme top/bottom)
        elif self.position.x == 190:
            if self.position.direction == Direction.TOP:
                # Target face: BOTTOM → subtract offset from y.
                return RobotPosition(
                    self.position.x - 10,
                    self.position.y + safety + base - offset,
                    Direction.BOTTOM
                )
            elif self.position.direction == Direction.BOTTOM:
                # Target face: TOP → add offset to y.
                return RobotPosition(
                    self.position.x - 10,
                    self.position.y - safety - base + offset,
                    Direction.TOP
                )
            elif self.position.direction == Direction.LEFT:
                # Target face: RIGHT → add offset to x.
                return RobotPosition(
                    self.position.x - safety - base + offset,
                    self.position.y,
                    Direction.RIGHT
                )
            else:  # Direction.RIGHT
                # Target face: LEFT → subtract offset from x.
                return RobotPosition(
                    self.position.x + safety + base - offset,
                    self.position.y,
                    Direction.LEFT
                )

        # All other locations
        else:
            if self.position.direction == Direction.TOP:
                # Target face: BOTTOM → subtract offset from y.
                return RobotPosition(
                    self.position.x,
                    self.position.y + safety + base - offset,
                    Direction.BOTTOM
                )
            elif self.position.direction == Direction.BOTTOM:
                # Target face: TOP → add offset to y.
                return RobotPosition(
                    self.position.x,
                    self.position.y - safety - base + offset,
                    Direction.TOP
                )
            elif self.position.direction == Direction.LEFT:
                # Target face: RIGHT → add offset to x.
                return RobotPosition(
                    self.position.x - safety - base + offset,
                    self.position.y,
                    Direction.RIGHT
                )
            else:
                # Target face: LEFT → subtract offset from x.
                return RobotPosition(
                    self.position.x + safety + base - offset,
                    self.position.y,
                    Direction.LEFT
                )

    def draw_obstacles(self, screen):
        # Draw the obstacle onto the grid.
        # We need to translate the obstacle's center into that with respect to PyGame
        # Get the coordinates of the grid's bottom left-hand corner.
        rect = pygame.Rect(0, 0, constants.OBSTACLE_LENGTH,
                           constants.OBSTACLE_LENGTH)
        rect.center = self.position.xy_pygame()
        pygame.draw.rect(screen, constants.RED, rect)

        # Draw the direction of the picture
        rect.width = constants.OBSTACLE_LENGTH / 2
        rect.height = constants.OBSTACLE_LENGTH / 2
        rect.center = self.position.xy_pygame()

        if self.position.direction == Direction.TOP:
            rect.centery -= constants.OBSTACLE_LENGTH / 4
        elif self.position.direction == Direction.BOTTOM:
            rect.centery += constants.OBSTACLE_LENGTH / 4
        elif self.position.direction == Direction.LEFT:
            rect.centerx -= constants.OBSTACLE_LENGTH / 4
        else:
            rect.centerx += constants.OBSTACLE_LENGTH / 4

        # Draw the picture place
        pygame.draw.rect(screen, constants.DARK_BLUE, rect)

    def draw_virtual_boundary(self, screen):
        # Get the boundary points
        points = self.get_boundary_points()

        # Draw left border
        pygame.draw.line(screen, constants.RED,
                         points[0].xy_pygame(), points[2].xy_pygame())
        # Draw right border
        pygame.draw.line(screen, constants.RED,
                         points[1].xy_pygame(), points[3].xy_pygame())
        # Draw upper border
        pygame.draw.line(screen, constants.RED,
                         points[2].xy_pygame(), points[3].xy_pygame())
        # Draw lower border
        pygame.draw.line(screen, constants.RED,
                         points[0].xy_pygame(), points[1].xy_pygame())

    def draw_robot_target(self, screen):
        target = self.get_robot_target_pos()

        rot_image = self.target_image
        angle = 0
        if target.direction == Direction.BOTTOM:
            angle = 180
        elif target.direction == Direction.LEFT:
            angle = 90
        elif target.direction == Direction.RIGHT:
            angle = -90

        rot_image = pygame.transform.rotate(rot_image, angle)
        rect = rot_image.get_rect()
        rect.center = target.xy_pygame()
        screen.blit(rot_image, rect)

    def draw(self, screen):
        # Draw the obstacle itself.
        self.draw_self(screen)
        # Draw the obstacle's boundary.
        self.draw_virtual_boundary(screen)
        # Draw the target for this obstacle.
        self.draw_robot_target(screen)
