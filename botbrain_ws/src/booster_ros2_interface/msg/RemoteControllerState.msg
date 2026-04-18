uint32 event  # SDL_EventType, value reference from SDL2/SDL_events.h
float32 lx # left stick horizontal direction, push left to -1, push right to 1
float32 ly # left stick vertical direction, push front to -1, push back to 1
float32 rx # right stick horizontal direction, push left to -1, push right to 1
float32 ry # right stick vertical direction, push front to -1, push back to 1
bool a
bool b
bool x
bool y
bool lb
bool rb
bool lt
bool rt
bool ls
bool rs
bool back
bool start

bool hat_c # Hat centered
bool hat_u # Hat up
bool hat_d # Hat down
bool hat_l # Hat left
bool hat_r # Hat right
bool hat_lu # Hat left up
bool hat_ld # Hat left down
bool hat_ru # Hat right up
bool hat_rd # Hat right down
uint8 hat_pos # Hat position, value reference from SDL2/SDL_joystick.h