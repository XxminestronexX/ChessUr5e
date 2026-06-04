#!/usr/bin/env python3
from spawn_utils import spawn_piece

def main():
    print("🐴 Spawning Cavalli...")
    pieces = [
        ('white_knight_1', 0, 1), ('white_knight_2', 0, 6),
        ('black_knight_1', 7, 1), ('black_knight_2', 7, 6)
    ]
    for name, c, r in pieces:
        is_black = (c > 4)
        color = '0.15 0.15 0.15 1' if is_black else '1 1 1 1'
        spawn_piece(name, c, r, 'cavallo_fide.stl', color, 'n', is_black)

if __name__ == '__main__':
    main()