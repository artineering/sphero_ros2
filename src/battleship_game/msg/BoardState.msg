# Message representing the current state of a board
# Published in response to display requests or game state changes

string player_id           # Unique identifier for the player
int32 rows                # Number of rows
int32 cols                # Number of columns
string[] grid             # Flattened grid data (row-major order)
string[] attacks          # List of all attacks (e.g., ["A1", "B5", "C3"])
string[] hits             # List of successful hits
string[] misses           # List of misses
string[] ships_sunk       # List of ship IDs that have been sunk
bool all_ships_sunk      # True if all ships are sunk
int32 total_attacks      # Total number of attacks made
int32 total_hits         # Total number of hits
int32 total_misses       # Total number of misses
float32 accuracy         # Hit accuracy (hits/total_attacks)
