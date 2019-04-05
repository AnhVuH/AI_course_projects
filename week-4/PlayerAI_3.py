from BaseAI_3 import BaseAI
import time
import math

possibleNewTiles = [2, 4]

class PlayerAI(BaseAI):

    def getMove(self, grid):
        depth = 0
        start = time.process_time()
        move, max_utility = self.maximize(grid,None,float("-inf"), float("inf"),depth, start)
        return move

    def maximize(self, grid,prev_move, alpha, beta,depth, start):
        end = time.process_time()
        process = start - end
        if depth >=4 or process >=0.1:
            return None, self.eval(grid)

        moves = grid.getAvailableMoves()
        if len(moves)==0:
            return prev_move, self.eval(grid)

        move,max_utility = (None,float("-inf"))

        for dir in moves:
            grid_copy = grid.clone()
            grid_copy.move(dir)

            cell, utility = self.minimize(grid_copy,prev_move, alpha, beta, depth + 1, start)

            if utility >= max_utility:
                move, max_utility = (dir, utility)
            if max_utility >= beta:
                break
            if max_utility > alpha:
                alpha = max_utility

        return move, max_utility

    def minimize(self, grid,prev_move, alpha, beta,depth, start):
        cell,min_utility = (None, float("inf"))
        cells = grid.getAvailableCells()

        prune = False
        for tile in possibleNewTiles:
            if prune:
                break
            for cell in cells:
                grid_copy = grid.clone()
                grid_copy.insertTile(cell, tile)

                move, utility = self.maximize(grid_copy,prev_move,alpha, beta, depth + 1,start)
                if utility < min_utility:
                    min_utility = utility
                if min_utility <= alpha:
                    prune = True
                    break
                if min_utility < beta:
                    beta = min_utility

        return cell, min_utility
    def eval(self,grid):
        if grid.getMaxTile() <=256:
            return  free_tiles( grid) + 1.1*monotonicity(grid) - smoothness(grid)
        elif grid.getMaxTile() ==512:
            return  2*free_tiles( grid) + 1.1*monotonicity(grid) - smoothness(grid) 
        elif grid.getMaxTile() ==1024:
            return  3*free_tiles(grid) + monotonicity(grid) - 1.5*smoothness(grid)
        else:
            return  3*free_tiles( grid) + 0.8*monotonicity(grid) - 1.7*smoothness(grid)

def free_tiles( grid):
    return len(grid.getAvailableCells())

def average_tile_numbers(grid):
    sum = 0
    count = 0
    for i in range(grid.size):
        for j in range(grid.size):
            tile = grid.map[i][j]
            if tile !=0 :
                sum += math.log2(tile)
                count +=1
    return sum/count

def monotonicity(grid):
    weight_matrix = [
        [2, 0.7, 0.2, 0.15],
        [0.7 , 0.2, 0.1, 0.07],
        [0.2 , 0.1, 0.07, 0.05],
        [0.15 , 0.07,  0.05, 0.03]
        ]
    count = 0
    for i in range(grid.size):
        for j in range(grid.size):
            if grid.map[i][j] !=0:
                count += math.log2(grid.map[i][j])* weight_matrix[i][j]
    return count


def smoothness(grid):
    total = 0
    for i in range(grid.size):
        for j in range(grid.size - 1):
            if grid.map[i][j] !=0 and grid.map[i][j+1] !=0:
                total += abs(math.log2(grid.map[i][j]) - math.log2(grid.map[i][j+1]))

    for i in range(grid.size):
        for j in range(grid.size - 1):
            if grid.map[j][i] !=0 and grid.map[j+1][i]:
                total += abs(math.log2(grid.map[j][i]) - math.log2(grid.map[j+1][i]))
    return total
