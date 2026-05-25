#!/usr/bin/env python3
from spawn_utils import spawn_piece

def main():
    print("🐴 Spawning Cavalli...")
    pieces = [
        ('white_knight_1', 1, 0), ('white_knight_2', 6, 0),
        ('black_knight_1', 1, 7), ('black_knight_2', 6, 7)
    ]
    for name, c, r in pieces:
        is_black = (r > 4)
        color = '0.15 0.15 0.15 1' if is_black else '1 1 1 1'
        spawn_piece(name, c, r, 'cavallo.stl', color, 'n', is_black)

if __name__ == '__main__':
    main()