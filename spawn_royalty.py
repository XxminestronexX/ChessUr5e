#!/usr/bin/env python3
from spawn_utils import spawn_piece

def main():
    print("👑 Spawning Regina e Re...")
    pieces = [
        ('white_queen', 0, 3, 'regina_fide.stl', '1 1 1 1', 'q', False),
        ('white_king', 0, 4, 're_fide.stl', '1 1 1 1', 'k', False),
        ('black_queen', 7, 3, 'regina_fide.stl', '0.15 0.15 0.15 1', 'q', True),
        ('black_king', 7, 4, 're_fide.stl', '0.15 0.15 0.15 1', 'k', True)
    ]
    for name, c, r, mesh, color, type, is_black in pieces:
        spawn_piece(name, c, r, mesh, color, type, is_black)

if __name__ == '__main__':
    main()