#!/usr/bin/env python3
from spawn_utils import spawn_piece

def main():
    print("⛪ Spawning Alfieri...")
    pieces = [
        ('white_bishop_1', 2, 0), ('white_bishop_2', 5, 0),
        ('black_bishop_1', 2, 7), ('black_bishop_2', 5, 7)
    ]
    for name, c, r in pieces:
        is_black = (r > 4)
        color = '0.15 0.15 0.15 1' if is_black else '1 1 1 1'
        spawn_piece(name, c, r, 'alfiere.stl', color, 'b', is_black)

if __name__ == '__main__':
    main()