#!/usr/bin/env python3
from spawn_utils import spawn_piece

def main():
    print("♟️ Spawning Pedoni...")
    # Bianchi (Riga 1, col 0-7)
    for i in range(8):
        spawn_piece(f'white_pawn_{i}', i, 1, 'pedone.stl', '1 1 1 1', 'p', False)
    # Neri (Riga 6, col 0-7)
    for i in range(8):
        spawn_piece(f'black_pawn_{i}', i, 6, 'pedone.stl', '0.15 0.15 0.15 1', 'p', True)

if __name__ == '__main__':
    main()