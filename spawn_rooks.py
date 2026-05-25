#!/usr/bin/env python3
from spawn_utils import spawn_piece

def main():
    print("🏰 Spawning Torri...")
    pieces = [
        ('white_rook_1', 0, 0), ('white_rook_2', 7, 0),
        ('black_rook_1', 0, 7), ('black_rook_2', 7, 7)
    ]
    for name, c, r in pieces:
        is_black = (r > 4)
        color = '0.15 0.15 0.15 1' if is_black else '1 1 1 1'
        spawn_piece(name, c, r, 'torre.stl', color, 'r', is_black)

if __name__ == '__main__':
    main()