"""Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).
"""
import folium
import functools
import networkx as nx
import osmnx as ox
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


# TODO: osmnx/networkxを廃止し、overpass api/OR-toolsのgraphを使う


class Vehicle(object):

    def __init__(self):
        self._capacity = 80
        self._speed = 25*1000/60/60   # 25km/h -> m/sec

    @property
    def capacity(self):
        return self._capacity

    @property
    def speed(self):
        return self._speed


class Data(object):

    def __init__(self, locations, demands):
        self.vehicle = Vehicle()
        self.num_vehicles = 3
        self.locations = locations
        self.demands = demands

        # デポを指定
        self.depot = 0

        # 店舗の時間指定
        self.time_windows = []
        for i in locations:
            time_range = i['time']
            # 00:00 時:分　を　秒に変換
            self.time_windows.append( \
                ((int(time_range[0][0:2]) * 3600) + (int(time_range[0][3:4]) * 60), \
                 (int(time_range[1][0:2]) * 3600) + (int(time_range[1][3:4]) * 60)) \
            )

        self.num_locations = len(locations)

        # 荷物一個あたりの積荷時間(秒)
        self.time_per_demand  = 60


def distance(x, y):
    """
    ノード間の距離をグラフから返す. コストは距離固定.
    :param x: (Int) 始点node id
    :param y: (Int) 終点node id
    :return: Int: 距離(m)
    """
    # print('x:{}, y:{}'.format(x,y))
    return nx.shortest_path_length(G, x, y, weight='length')


class CreateDistanceCallback(object):
    """二点間の距離を返すCallbackを定義"""

    def __init__(self, data):
        """コストマトリックスを生成"""
        size = data.num_locations
        self.matrix = {}

        for from_node in range(size):
            self.matrix[from_node] = {}
            for to_node in range(size):
                x = data.locations[from_node]['nearest_node']
                y = data.locations[to_node]['nearest_node']
                self.matrix[from_node][to_node] = distance(x, y)


    @functools.lru_cache()
    def Distance(self, from_node, to_node):
        """
        ノード間の距離を生成したマトリックスから返す
        ソルバーはIntしか取らないので、Intに変換してから返す
        """
        return int(self.matrix[from_node][to_node])



class CreateDemandCallback(object):
    """現在のノードの需要を返す"""
    def __init__(self, data):
        self.demands = data.demands

    def demand_callback(self, from_node, to_node):
        del to_node
        return self.demands[from_node]

def add_capacity_constraints(routing, data, demand_callback):
    """"積荷上限数を制約に追加する"""
    cap = 'Capacity'
    routing.AddDimension(demand_callback, 0, data.vehicle.capacity, True, cap)

class CreateTimeCallback(object):
    """店舗間の合計時間を返すコールバックを定義"""

    def service_time(self, data, node):
        """拠点ごとの荷積時間を返す。荷積時間は単純に個数x1個あたりの積荷時間"""
        return data.demands[node] * data.time_per_demand

    def travel_time(self, data, from_node, to_node):
        """店舗間の移動時間を返す"""
        if from_node == to_node:
            travel_time = 0
        else:
            travel_time = distance( \
                            data.locations[from_node]['nearest_node'], \
                            data.locations[to_node]['nearest_node']  \
                        ) / data.vehicle.speed
        return travel_time

    def __init__(self, data):
        """合計時間matrixを初期化"""
        self.total_time = {}
        for from_node in range(data.num_locations):
            self.total_time[from_node] = {}
            for to_node in range(data.num_locations):
                if from_node == to_node:
                    self.total_time[from_node][to_node] = 0
                else:
                    self.total_time[from_node][to_node] = int(
                        self.service_time(data, from_node) + self.travel_time(data, from_node, to_node)
                    )

    def time_evaluator(self, from_node, to_node):
        """ノード間の合計時間を返す"""
        return self.total_time[from_node][to_node]


def add_time_window_constraints(routing, data, time_evaluator):
    """全体の走行時間の制約を追加する"""
    time = "Time"
    horizon = 14400 # 4時間 -> 秒
    routing.AddDimension(
        time_evaluator,
        horizon, # 待ち時間の上限
        horizon, # 走行時間の上限
        True,
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    for location_idx, time_window in enumerate(data.time_windows):
    #     time_dimension.CumulVar(location_idx).SetRange(time_window[0],time_window[1])
          time_dimension.CumulVar(location_idx).SetMin(time_window[0])
          time_dimension.CumulVar(location_idx).SetMax(time_window[1])


class ConsolePrinter(object):
    """コンソールに結果を表示"""
    def __init__(self, data, routing, assignment):
        self._data = data
        self._routing = routing
        self._assignment = assignment
        # 車両の色
        self.vehicle_color = ['red', 'blue', 'green', 'orange', 'yellow']


    @property
    def data(self):
        return self._data

    @property
    def routing(self):
        return self._routing

    @property
    def assignment(self):
        return self._assignment

    def print(self):
        capacity_dimension = self.routing.GetDimensionOrDie('Capacity')
        time_dimension = self.routing.GetDimensionOrDie('Time')
        total_dist = 0
        total_time = 0

        if self.assignment:
            # 車両ごと
            for vehicle_id in range(self.data.num_vehicles):
                index = self.routing.Start(vehicle_id)
                plan_output = '車両番号 {0}({1}):\n'.format(vehicle_id, self.vehicle_color[vehicle_id])
                route_dist = 0
                depo_node_index = self.routing.IndexToNode(index)
                depo_node = self.data.locations[depo_node_index]


                # ルートの始点終点を黒アイコンで表示
                folium.Marker(
                    [depo_node['lat'], depo_node['lon']],
                    popup=folium.Popup("{}({})".format(depo_node['name'], depo_node_index), parse_html=True),
                    icon=folium.Icon(color='black')
                ).add_to(map)

                # 各車両のルート
                while not self.routing.IsEnd(index):
                    node_index = self.routing.IndexToNode(index)
                    next_node_index = self.routing.IndexToNode(
                        self.assignment.Value(self.routing.NextVar(index)))
                    node_dist = distance(
                        self.data.locations[node_index]['nearest_node'],
                        self.data.locations[next_node_index]['nearest_node'])
                    route_dist += node_dist
                    load_var = capacity_dimension.CumulVar(index)
                    route_load = self.assignment.Value(load_var)
                    time_var = time_dimension.CumulVar(index)
                    time_min = self.assignment.Min(time_var)
                    time_max = self.assignment.Max(time_var)
                    plan_output += ' [{0} 積載数{1}+{2} {3}着] --|{4}m|-->'.format(self.data.locations[node_index]['name'], \
                                                                                  route_load,  \
                                                                                  self.data.demands[node_index], \
                                                                                  secToHourMin(time_min), round(node_dist))
                    index = self.assignment.Value(self.routing.NextVar(index))
                    start_node = self.data.locations[node_index]
                    end_node = self.data.locations[next_node_index]


                    if start_node != end_node:
                        # ノードアイコンとノード間のルート線を地図にプロット
                        folium_route = nx.shortest_path(G, start_node['nearest_node'], end_node['nearest_node'], weight='length')
                        ox.plot_route_folium(G, folium_route, route_map=map, route_color=self.vehicle_color[vehicle_id], route_width=8, route_opacity=0.8)
                        folium.Marker(
                            [end_node['lat'], end_node['lon']],
                            popup=folium.Popup("{0} {1}着 積載数{2}+{3}".format(end_node['name'], \
                                                                            secToHourMin(time_min), \
                                                                            route_load, \
                                                                            self.data.demands[node_index] \
                                                                            ), parse_html=True),
                            icon=folium.Icon(color=self.vehicle_color[vehicle_id])
                        ).add_to(map)

                if end_node != depo_node:
                    # 最後のノードとデポまでの帰路をプロット
                    folium_route = nx.shortest_path(G, end_node['nearest_node'], depo_node['nearest_node'], weight='length')
                    ox.plot_route_folium(G, folium_route, route_map=map, route_color=self.vehicle_color[vehicle_id], route_opacity=0.5)

                node_index = self.routing.IndexToNode(index)
                load_var = capacity_dimension.CumulVar(index)
                route_load = self.assignment.Value(load_var)
                time_var = time_dimension.CumulVar(index)
                route_time = self.assignment.Value(time_var)
                time_min = self.assignment.Min(time_var)
                time_max = self.assignment.Max(time_var)
                total_dist += route_dist
                total_time += route_time
                plan_output += ' {0} 積載数({1}) {2}着\n'.format(end_node['name'], route_load, secToHourMin(time_min))
                plan_output += ' - ルート距離　　: {0}m\n'.format(round(route_dist,2))
                plan_output += ' - ルート荷物数　: {0}\n'.format(route_load)
                plan_output += ' - ルート所要時間: {0}\n'.format(secToHourMinSecStr(route_time))
                print(plan_output)
            print('全ルート総距離: {0}m'.format(round(total_dist, 2)))
            print('全ルート総時間: {0}'.format(secToHourMinSecStr(total_time)))
        else:
            print('解が見つかりませんでした')


def secToHourMinSec(sec):
    """秒を時:分に変換"""
    if sec >= 3600:
        hour = sec//3600
        min = sec%3600//60
        sec = (sec - hour*3600 - min*60)
    else:
        hour = 0
        min = sec//60
        sec = (sec - min*60)
    return "{0:02d}:{1:02d}:{2:02d}".format(hour,min,sec)

def secToHourMin(sec):
    """秒を時:分に変換"""
    if sec >= 3600:
        hour = sec//3600
        min = sec%3600//60
    else:
        hour = 0
        min = sec//60
    return "{0:02d}:{1:02d}".format(hour,min)

def secToHourMinSecStr(sec):
    """秒を時:分に変換"""
    if sec >= 3600:
        hour = sec//3600
        min = sec%3600//60
        sec = (sec - hour*3600 - min*60)
    else:
        hour = 0
        min = sec//60
        sec = (sec - min*60)
    return "{0:02d}時間{1:02d}分{1:02d}秒".format(hour,min,sec)



class ShopData(object):

    def __init__(self):

       self.locations = [ \
        {'nearest_node': 1809615572, 'time':('00:00', '00:00'), 'name': 'アークヒルズ店', 'lat': 35.6681770, 'lon': 139.7397242},  # ここがDepo
        {'nearest_node': 251864110,  'time':('00:00', '00:30'), 'name': '港赤坂九丁目店', 'lat': 35.6680961, 'lon': 139.7328141},
        {'nearest_node': 1760515775, 'time':('00:00', '02:00'), 'name': '城山トラストタワー', 'lat': 35.6649108, 'lon': 139.7431470},
        {'nearest_node': 2162858601, 'time':('00:00', '00:50'), 'name': '合同庁舎第７号館', 'lat': 35.6715977, 'lon': 139.7483260},
        {'nearest_node': 499193143,  'time':('00:00', '02:00'), 'name': '東麻布三丁目', 'lat': 35.6571622, 'lon': 139.7392354},
        {'nearest_node': 1618521241, 'time':('00:00', '02:00'), 'name': '虎ノ門一丁目', 'lat': 35.6685010, 'lon': 139.7489690},
        {'nearest_node': 499189994,  'time':('00:00', '02:00'), 'name': '赤坂六丁目店', 'lat': 35.6700138, 'lon': 139.7337181},
        {'nearest_node': 2207933301, 'time':('00:00', '02:00'), 'name': '100 元麻布店', 'lat': 35.6578560, 'lon': 139.7274620},
        {'nearest_node': 499192852,  'time':('00:00', '02:00'), 'name': '麻布十番一丁目店', 'lat': 35.657044, 'lon': 139.736167},
        {'nearest_node': 1655440289, 'time':('00:00', '02:00'), 'name': '西麻布店', 'lat': 35.6603463, 'lon': 139.7230918},
        {'nearest_node': 4414312668, 'time':('00:00', '02:00'), 'name': '赤坂氷川公園前店', 'lat': 35.6713681, 'lon': 139.7372166},
        {'nearest_node': 1655503992, 'time':('00:00', '02:00'), 'name': '虎ノ門琴平点', 'lat': 35.6703080, 'lon': 139.7487411},
        {'nearest_node': 1482491357, 'time':('00:00', '02:00'), 'name': '新橋六丁目店', 'lat': 35.661263, 'lon': 139.754532}]


       # 店舗ごとの集荷需要
       self.demands = [1, 19, 21, 6, 19, 7, 12, 16, 6, 16, 8, 14, 21]


def main():

    # 表示地図を定義
    global G, map
    ROPPONGI = (35.663236, 139.732275)
    G = ox.graph_from_point(ROPPONGI, distance=3000, network_type='drive', simplify=False)
    map = folium.Map(location=ROPPONGI, tiles='cartodbpositron', zoom_start=7)

    shops = ShopData()
    data = Data(shops.locations, shops.demands)

    # ルーティングモデルを生成
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    # 辺のウェイトを定義
    distance_evaluator = CreateDistanceCallback(data).Distance
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)

    # 積載数の制約を追加
    demand_evaluator = CreateDemandCallback(data).demand_callback
    add_capacity_constraints(routing, data, demand_evaluator)

    # 指定時間の制約を追加
    time_evaluator = CreateTimeCallback(data).time_evaluator
    add_time_window_constraints(routing, data, time_evaluator)

    # ルーティングモデルのパラメータ設定
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    # 探索処理時間の上限(ms)
    search_parameters.time_limit_ms = 30000

    # 局所探索のためのヒューリスティックモデルを指定する
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # 解を求める
    assignment = routing.SolveWithParameters(search_parameters)

    # 結果を表示
    printer = ConsolePrinter(data, routing, assignment)
    printer.print()

    # map.save("ortools_cvrptw.html")
    map

if __name__ == "__main__":

    main()


