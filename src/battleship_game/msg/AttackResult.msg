# Message representing the result of an attack

string attacker_id         # Player who made the attack
string target_id           # Player who was attacked
string coordinate          # Attack coordinate
bool is_hit               # True if attack hit a ship
bool already_attacked     # True if coordinate was already attacked
bool ship_sunk           # True if the hit sank a ship
string ship_id           # ID of ship that was hit/sunk (empty if miss)
bool game_over           # True if this attack ended the game
string winner_id         # ID of winner if game is over (empty otherwise)
