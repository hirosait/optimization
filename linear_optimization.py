###
# 01ナップサック問題
#
# ナップサックは最大7kgまで入れられる
# ぬいぐるみは４種類ある
# 極小 16万円 2kg
# 小   19万円 3kg
# 中   23万円 4kg
# 大   28万円 5kg
# なるべく価値の高いぬいぐるみを持ち出すには、ナップサックにどれを入れるか
#
# 定式化
#
# 持っていくときは１，持っていかないときは０ -> バイナリ変数
# 目的関数：16x1 + 19x2 + 23x3 + 28x4 （最大化）
# 制約条件：2x1 + 3x2 + 4x3 + 5x4 <= 7
# 変数：x1, x2, x3, x4 ∈ {0,1}
#
#
from ortools.algorithms import pywrapknapsack_solver

def main():
    solver = pywrapknapsack_solver.KnapsackSolver(
        pywrapknapsack_solver.KnapsackSolver.KNAPSACK_DYNAMIC_PROGRAMMING_SOLVER, 'test'
    )

    values = [16,19,23,28]
    weights = [[2,3,4,5]]
    capacity = [7]
    solver.Init(values, weights, capacity)
    computed_value = solver.Solve()

    packed_items = [x for x in range(0, len(weights[0]))
                    if solver.BestSolutionContains(x)]
    packed_weights = [weights[0][i] for i in packed_items]

    print("持ち出しアイテム：", packed_items)
    print("持ち出すアイテムの重さ： ", packed_weights)
    print("持ち出しアイテムの合計の価値：", computed_value)

if __name__ == '__main__':
    main()

