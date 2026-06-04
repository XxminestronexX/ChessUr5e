#!/usr/bin/env python3
from spawn_utils import spawn_piece

def main():
    print("⛪ Spawning Alfieri...")
    pieces = [
        ('white_bishop_1', 0, 2), ('white_bishop_2', 0, 5),
        ('black_bishop_1', 7, 2), ('black_bishop_2', 7, 5)
    ]
    for name, c, r in pieces:
        is_black = (c > 4)
        color = '0.15 0.15 0.15 1' if is_black else '1 1 1 1'
        spawn_piece(name, c, r, 'alfiere_fide.stl', color, 'b', is_black)

if __name__ == '__main__':
    main()