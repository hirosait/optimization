###
# 線形最適化　練習問題
#
# 豚丼、鶏丼、ミックス丼の３種類がある。
# 豚丼を作るには、200gの豚肉と100gの鶏肉
# 鶏丼を作るには、100gの豚丼と200gの鶏肉
# ミックス丼は豚、鶏、牛をそれぞれ100g
# 豚と鶏はそれぞれ6kg, 牛は3kgしかない
# 価格は豚丼1500円、鶏丼1800円、ミックス丼3000円
# 利益を最大化するには、丼を何杯ずつ作ればよいのだろうか？
#
#
# 定式化
#
# 変数：豚丼 x1, 鶏丼 x2, ミックス丼 x3
# 目的関数：15x1 + 18x2 + 30x3
# 制約=肉量の上限
# 　豚：2x1 + x2 + x3 <= 60
# 　鶏：x1 + 2x2 + x3 <= 60
# 　牛：x3 <= 30
##

from ortools.linear_solver import pywraplp


def main():

    solver = pywraplp.Solver('線形最適化問題', pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)

    # 変数を定義
    x1 = solver.NumVar(-solver.infinity(), solver.infinity(), 'x1')
    x2 = solver.NumVar(-solver.infinity(), solver.infinity(), 'x2')
    x3 = solver.NumVar(-solver.infinity(), solver.infinity(), 'x3')

    # 制約を定義
    constraint1 = solver.Constraint(-solver.infinity(), 60)
    constraint1.SetCoefficient(x1, 2)
    constraint1.SetCoefficient(x2, 1)
    constraint1.SetCoefficient(x3, 1)

    constraint2 = solver.Constraint(-solver.infinity(), 60)
    constraint2.SetCoefficient(x1, 1)
    constraint2.SetCoefficient(x2, 2)
    constraint2.SetCoefficient(x3, 1)

    constraint3 = solver.Constraint(-solver.infinity(), 30)
    constraint3.SetCoefficient(x3, 1)

    # 目的関数の設定
    objective = solver.Objective()
    objective.SetCoefficient(x1, 15)
    objective.SetCoefficient(x2, 18)
    objective.SetCoefficient(x3, 30)

    objective.SetMaximization()

    solver.Solve()

    opt_solution = 15 * x1.solution_value() + 18 * x2.solution_value() + 30 * x3.solution_value()
    print('変数の数：{}'.format(solver.NumVariables()))
    print('制約の数：{}'.format(solver.NumConstraints()))
    print('==最適解==')
    print('x1 = ', x1.solution_value())
    print('x2 = ', x2.solution_value())
    print('x3 = ', x3.solution_value())
    print('目的関数値 =', opt_solution)


if __name__ == '__main__':
    main()

