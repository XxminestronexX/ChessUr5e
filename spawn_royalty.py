#!/usr/bin/env python3
from spawn_utils import spawn_piece

def main():
    print("👑 Spawning Regina e Re...")
    pieces = [
        ('white_queen', 3, 0, 'regina.stl', '1 1 1 1', 'q', False),
        ('white_king', 4, 0, 're.stl', '1 1 1 1', 'k', False),
        ('black_queen', 3, 7, 'regina.stl', '0.15 0.15 0.15 1', 'q', True),
        ('black_king', 4, 7, 're.stl', '0.15 0.15 0.15 1', 'k', True)
    ]
    for name, c, r, mesh, color, type, is_black in pieces:
        spawn_piece(name, c, r, mesh, color, type, is_black)

if __name__ == '__main__':
    main()