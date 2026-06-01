import random
import tkinter as tk
from enum import Enum
from typing import List, Tuple
from dataclasses import dataclass
import numpy as np

OBSTACLE_MULTIPLIER_MIN = 15
OBSTACLE_MULTIPLIER_MAX = 20
MIN_MAP_SIZE = 9
MAX_MAP_SIZE = 14

class SimulationState(Enum):
    RUNNING = "running"
    CRASHED = "crashed"
    GOAL_REACHED = "goal_reached"

class Orientation(Enum):
    NORTH = ("↑", 0, -1)
    SOUTH = ("↓", 0, 1)
    EAST = ("→", 1, 0)
    WEST = ("←", -1, 0)

@dataclass
class GameState:
    perception_matrix: np.ndarray  # Matrix of confidence values (0, 0.25, 0.5, 0.75, 1)
    current_position: Tuple[int, int]
    current_orientation: Orientation
    simulation_state: SimulationState

class Robot:
    def __init__(self, x: int, y: int, orientation: Orientation):
        self.x = x
        self.y = y
        self.orientation = orientation
        
    def get_perception_confidence(self, is_real_obstacle: bool) -> float:
        rand = random.random()
        
        if rand < 0.2:  # 20% chance for unknown (0.5)
            return 0.5
        elif rand < 0.6:  # 40% chance for uncertain perception (0.25 or 0.75)
            if is_real_obstacle:
                return 0.75 if random.random() < 0.75 else 0.25
            else:
                return 0.25 if random.random() < 0.75 else 0.75
        else:  # 40% chance for perfect perception (0 or 1)
            return 1.0 if is_real_obstacle else 0.0

    def update_vision(self, world_map: List[List[str]]) -> np.ndarray:
        height, width = len(world_map), len(world_map[0])
        perception_matrix = np.full((height, width), 0.5)  # Initialize with unknown
        
        # Current position is always perfectly known
        perception_matrix[self.y, self.x] = 0.0
        
        dx, dy = self.orientation.value[1], self.orientation.value[2]
        
        # Front vision (3 cells)
        for i in range(1, 4):
            front_x = self.x + (dx * i)
            front_y = self.y + (dy * i)
            
            if 0 <= front_x < width and 0 <= front_y < height:
                if (front_x == 0 or front_x == width-1 or
                    front_y == 0 or front_y == height-1):
                    perception_matrix[front_y, front_x] = 1.0
                    break
                elif world_map[front_y][front_x] == 'E':
                    perception_matrix[front_y, front_x] = 2.0  # Changed from 0.0 to 2.0
                else:
                    is_real_obstacle = world_map[front_y][front_x] == 'X'
                    perception_matrix[front_y, front_x] = self.get_perception_confidence(is_real_obstacle)
                
                if world_map[front_y][front_x] == 'X':
                    break
            else:
                break
        
        # Side vision (1 cell on each side)
        for side_dx, side_dy in [(-dy, dx), (dy, -dx)]:
            side_x = self.x + side_dx
            side_y = self.y + side_dy
            if 0 <= side_x < width and 0 <= side_y < height:
                if (side_x == 0 or side_x == width-1 or
                    side_y == 0 or side_y == height-1):
                    perception_matrix[side_y, side_x] = 1.0
                elif world_map[side_y][side_x] == 'E':
                    perception_matrix[side_y, side_x] = 2.0  # Changed from 0.0 to 2.0
                else:
                    is_real_obstacle = world_map[side_y][side_x] == 'X'
                    perception_matrix[side_y, side_x] = self.get_perception_confidence(is_real_obstacle)
        
        return perception_matrix

class Simulator:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Robot Navigation Simulator")
        self.cell_size = 30  # Reduced cell size to fit both maps
        
        # Create frame to hold both canvases
        self.frame = tk.Frame(self.root)
        self.frame.pack(padx=20, pady=20)
        
        # Labels for the maps
        tk.Label(self.frame, text="True Map", font=('TkDefaultFont', 12, 'bold')).grid(row=0, column=0, pady=5)
        tk.Label(self.frame, text="Robot's Perception", font=('TkDefaultFont', 12, 'bold')).grid(row=0, column=1, pady=5)
        
        self.status_label = tk.Label(self.root, text="Simulation Running...")
        self.status_label.pack(pady=5)

        # Add label for step count
        self.steps_label = tk.Label(self.root, text="Steps: 0", font=('TkDefaultFont', 12, 'bold'))
        self.steps_label.pack(pady=5)
        
        # Add reset button
        self.reset_button = tk.Button(self.root, text="Reset Simulation", command=self.generate_new_simulation)
        self.reset_button.pack(pady=5)
        
        self.quit_button = tk.Button(self.root, text="Quit", command=self.quit_simulation)
        self.quit_button.pack(pady=5)
        
        self.true_canvas = None
        self.perception_canvas = None
        self.legend_frame = None  # Add reference to legend frame
        
        #initialise everything
        self.cells = {"true": [], "perception": []}
        self.robot = None
        self.goal_pos = None
        self.map = None
        self.state = SimulationState.RUNNING
        self.moves_made = 0
        self.isquit = False
        
        self.draw_legend()
        self.map, self.robot, self.goal_pos = self._generate_map()
        self.state = SimulationState.RUNNING
        self.moves_made = 0
        self.current_perception_matrix = None
        # Update the steps display
        self.steps_label.config(text="Steps: 0")
        # Reset the status message
        self.status_label.config(text="Simulation Running...")
        # Clear and recreate widgets
        self.create_widgets()

    def quit_simulation(self):
        print("Quitting 1/3 ...")
        self.isquit = True

    def attempt_quit(self):
        if self.isquit:
            print("Quitting 2/3 ...")
            return True

    def get_game_state(self) -> GameState:
        # Only update perception if we haven't already done so
        if self.current_perception_matrix is None:
            self.current_perception_matrix = self.robot.update_vision(self.map)
        
        return GameState(
            perception_matrix=self.current_perception_matrix,
            current_position=(self.robot.x, self.robot.y),
            current_orientation=self.robot.orientation,
            simulation_state=self.state
        )
    

    def step(self, action: str) -> GameState:
        """Execute one step of the simulation based on the action."""



        if action not in ['N', 'S', 'E', 'W', 'M']:
            raise ValueError("Invalid action. Must be one of: N, S, E, W, M")
        
        self.current_perception_matrix = None
        self.moves_made += 1
        
        if action == 'M':
            dx, dy = self.robot.orientation.value[1], self.robot.orientation.value[2]
            new_x = self.robot.x + dx
            new_y = self.robot.y + dy
            
            if (0 <= new_x < len(self.map[0]) and 
                0 <= new_y < len(self.map) and 
                self.map[new_y][new_x] != 'X'):
                self.robot.x = new_x
                self.robot.y = new_y
                
                if (self.robot.x, self.robot.y) == self.goal_pos:
                    self.state = SimulationState.GOAL_REACHED
                    self.status_label.config(text=f"Goal reached in {self.moves_made} moves!")

            else:
                self.state = SimulationState.CRASHED
                self.status_label.config(text="Robot crashed. Game over.")


        else:
            if action == 'N':
                self.robot.orientation = Orientation.NORTH
            elif action == 'S':
                self.robot.orientation = Orientation.SOUTH
            elif action == 'E':
                self.robot.orientation = Orientation.EAST
            elif action == 'W':
                self.robot.orientation = Orientation.WEST
        
        self.steps_label.config(text=f"Steps: {self.moves_made}")
        self.draw_map() 
            
        return self.get_game_state()

    def generate_new_simulation(self):
        self.draw_legend()
        self.map, self.robot, self.goal_pos = self._generate_map()
        self.state = SimulationState.RUNNING
        self.moves_made = 0
        self.current_perception_matrix = None
        # Update the steps display
        self.steps_label.config(text="Steps: 0")
        # Reset the status message
        self.status_label.config(text="Simulation Running...")
        # Clear and recreate widgets
        self.create_widgets()

    # MAP GENERATION LOGIC ALL HERE
    def _generate_map(self):
        map_size = random.randint(MIN_MAP_SIZE, MAX_MAP_SIZE)
        world_map = [['X' for _ in range(map_size)] for _ in range(map_size)]
        
        # Clear inner cells
        for y in range(1, map_size-1):
            for x in range(1, map_size-1):
                world_map[y][x] = ' '
        
        # Add random obstacles
        obstacle_multiplier = random.randint(OBSTACLE_MULTIPLIER_MIN, OBSTACLE_MULTIPLIER_MAX)
        num_obstacles =  map_size * map_size * obstacle_multiplier // 100
        for _ in range(num_obstacles):
            while True:
                x, y = random.randint(1, map_size-2), random.randint(1, map_size-2)
                if world_map[y][x] == ' ':
                    world_map[y][x] = 'X'
                    break
        
        # Place robot
        robot_x, robot_y = random.randint(1, map_size-2), random.randint(1, map_size-2)
        while world_map[robot_y][robot_x] == 'X':
            robot_x, robot_y = random.randint(1, map_size-2), random.randint(1, map_size-2)
        
        initial_orientation = random.choice(list(Orientation))
        robot = Robot(robot_x, robot_y, initial_orientation)
        
        # Place goal
        while True:
            goal_x, goal_y = random.randint(1, map_size-2), random.randint(1, map_size-2)
            if (goal_x, goal_y) != (robot_x, robot_y) and world_map[goal_y][goal_x] != 'X':
                if abs(goal_x - robot_x) + abs(goal_y - robot_y) >= map_size // 2:
                    world_map[goal_y][goal_x] = 'E'
                    break
        
        return world_map, robot, (goal_x, goal_y)

    # MAP DRAWING LOGIC
    def draw_map(self):
        if self.current_perception_matrix is None:
            self.current_perception_matrix = self.robot.update_vision(self.map)
        
        perception_matrix = self.current_perception_matrix
        self.cells = {"true": [], "perception": []}
        
        def get_perception_color(perception_value: float) -> str:
            if perception_value == 2.0:  # Add this line to handle goal tiles
                return 'green'
            elif perception_value == 0.5:
                return 'black'
            elif perception_value == 0.75:
                return '#800000'  # Dark red
            elif perception_value == 0.25:
                return '#808080'  # Gray
            elif perception_value == 1.0:
                return 'red'
            else:  # perception_value == 0.0
                return 'white'

        def get_true_color(cell_value: str) -> str:
            if cell_value == 'X':
                return 'red'
            elif cell_value == 'E':
                return 'green'
            else:
                return 'white'

        # Clear existing canvases instead of recreating them
        self.true_canvas.delete("all")
        self.perception_canvas.delete("all")

        # Draw true map
        for i in range(len(self.map)):
            row = []
            for j in range(len(self.map)):
                x1 = j * self.cell_size
                y1 = i * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                color = get_true_color(self.map[i][j])
                cell = self.true_canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='black')
                
                # Draw robot and symbols on true map
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                if self.map[i][j] == 'E':
                    self.true_canvas.create_text(center_x, center_y, text='E', 
                                              font=('TkDefaultFont', 14), fill='white')
                
                if i == self.robot.y and j == self.robot.x:
                    self.true_canvas.create_text(center_x, center_y, 
                                              text=self.robot.orientation.value[0],
                                              font=('TkDefaultFont', 14), fill='blue')
                
                row.append(cell)
            self.cells["true"].append(row)

        # Draw perception map
        for i in range(len(self.map)):
            row = []
            for j in range(len(self.map)):
                x1 = j * self.cell_size
                y1 = i * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                color = get_perception_color(perception_matrix[i, j])
                
                if i == self.robot.y and j == self.robot.x:
                    color = 'white'
                elif self.map[i][j] == 'E' and perception_matrix[i, j] != 0.5:
                    color = 'green'
                
                cell = self.perception_canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='black')
                
                # Draw robot and known goal on perception map
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                if self.map[i][j] == 'E' and perception_matrix[i, j] != 0.5:
                    self.perception_canvas.create_text(center_x, center_y, text='E', 
                                                    font=('TkDefaultFont', 14), fill='white')
                
                if i == self.robot.y and j == self.robot.x:
                    self.perception_canvas.create_text(center_x, center_y, 
                                                    text=self.robot.orientation.value[0],
                                                    font=('TkDefaultFont', 14), fill='blue')
                
                row.append(cell)
            self.cells["perception"].append(row)
        
        self.root.update()            

    def draw_legend(self):
        # Remove any existing legend first
        if self.legend_frame:
            self.legend_frame.destroy()
                
        self.legend_frame = tk.Frame(self.root)
        self.legend_frame.pack(pady=10)
        
        # True map legend
        true_legend = tk.LabelFrame(self.legend_frame, text="True Map Legend", padx=5, pady=5)
        true_legend.pack(side=tk.LEFT, padx=10)
        
        items = [
            ("Free Space", "white"),
            ("Obstacle", "red"),
            ("Goal", "green")
        ]
        
        for text, color in items:
            frame = tk.Frame(true_legend)
            frame.pack(anchor='w')
            tk.Canvas(frame, width=15, height=15, bg=color).pack(side=tk.LEFT, padx=5)
            tk.Label(frame, text=text).pack(side=tk.LEFT)
        
        # Perception map legend
        perception_legend = tk.LabelFrame(self.legend_frame, text="Robot's Perception Legend", padx=5, pady=5)
        perception_legend.pack(side=tk.LEFT, padx=10)
        
        items = [
            ("0% chance of obstacle", "white"),
            ("25% chance of obstacle", "#808080"),
            ("50% chance of obstacle", "black"),
            ("75% chance of obstacle", "#800000"),
            ("100% chance of obstacle", "red"),
            ("Known Goal", "green")
        ]
        
        for text, color in items:
            frame = tk.Frame(perception_legend)
            frame.pack(anchor='w')
            tk.Canvas(frame, width=15, height=15, bg=color).pack(side=tk.LEFT, padx=5)
            tk.Label(frame, text=text).pack(side=tk.LEFT)

    def create_widgets(self):
        if self.true_canvas:
            self.true_canvas.destroy()
        if self.perception_canvas:
            self.perception_canvas.destroy()
        
        map_size = len(self.map) * self.cell_size
        
        # Create true map canvas
        self.true_canvas = tk.Canvas(self.frame, width=map_size, height=map_size)
        self.true_canvas.grid(row=1, column=0, padx=10)
        
        # Create perception map canvas
        self.perception_canvas = tk.Canvas(self.frame, width=map_size, height=map_size)
        self.perception_canvas.grid(row=1, column=1, padx=10)
        
        self.draw_map()

def create_simulator() -> Simulator:
    """Creates and returns a new simulator instance."""
    return Simulator()