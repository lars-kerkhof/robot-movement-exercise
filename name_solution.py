# DO NOT MODIFY IMPORT
from robot_simulator import create_simulator, GameState
from time import sleep

# DO NOT MODIFY FUNCTION DECLARATION
def choose_action(game_state: GameState) -> str:
    """
    Choose the next action based on the current game state.
    
    The game state contains:
        - perception_matrix: View of the map (numpy array) where:
            0.0 = 0% of being an obstacle
            0.25 = 25% chance of being an obstacle
            0.5 = 50% chance of being an obstacle
            0.75 = 75% chance of being an obstacle
            1.0 = 100% chance of being an obstacle
        - current_position: Your position as (x, y)
        - current_orientation: Which way you're facing ('NORTH', 'SOUTH', 'EAST', 'WEST')
        - simulation_state: Game state ('RUNNING', 'CRASHED', 'GOAL_REACHED')
    
    Your Returns:
        str: One of these actions:
            'N' = turn North
            'S' = turn South
            'E' = turn East
            'W' = turn West
            'M' = move forward
    """
    #Example utilisation of game_state:
    # if game_state.current_orientation.name == "NORTH":
    # if game_state.current_position[1] == 5:
    # if game_state.perception_matrix[3][3] == 0.5:
    # if game_state.simulation_state.name == 'RUNNING':

    #TODO: YOUR CODE HERE!
    #Example algorithm:
    if game_state.current_orientation.name != "NORTH":
        return 'N'
    else:
        return 'M'




#DO NOT MODIFY CODE BEYOND THIS POINT (except the 2nd sleep func)
def main():
    simulator = create_simulator()
    game_state = simulator.get_game_state()
    last_moves = -1  # Track the previous moves_made count
    
    while True:
        if simulator.attempt_quit():
            print("Quitting 3/3 ...")
            quit()
            
        # Detect reset by checking if moves_made decreased
        if simulator.moves_made < last_moves:
            sleep(0.8)  # Small delay to ensure clean state
            game_state = simulator.get_game_state()
            
        last_moves = simulator.moves_made
            
        if game_state.simulation_state.name == "RUNNING":
            action = choose_action(game_state)
            game_state = simulator.step(action)
            
        sleep(0.5)  # You may modify this if you want to speed up your simulation
        simulator.root.update()  # Ensure Tkinter processes events even when game is not running

if __name__ == "__main__":
    main()