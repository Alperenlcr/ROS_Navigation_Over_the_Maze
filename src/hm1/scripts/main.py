#! /usr/bin/python3.8
import time

from utilss import *


t1.start()

graph = Graph()

# graph.nodes.append(['s', 1.125, 1.095, 7, 180, 270])

# graph.nodes.append(['r', 2.2, 2.312, 2, 180, 270])
# graph.add_edge(to_coordinate(graph.nodes[-2][1], graph.nodes[-2][2]), to_coordinate(graph.nodes[-1][1], graph.nodes[-1][2]), graph.nodes[-2][3])
# graph.nodes.append(['b', 2, 1, 5, 180, 270])
# graph.add_edge(to_coordinate(graph.nodes[-2][1], graph.nodes[-2][2]), to_coordinate(graph.nodes[-1][1], graph.nodes[-1][2]), graph.nodes[-2][3])
# graph.nodes.append(['r', 3.234, 3.312, 3, 180, 270])
# graph.add_edge(to_coordinate(graph.nodes[-2][1], graph.nodes[-2][2]), to_coordinate(graph.nodes[-1][1], graph.nodes[-1][2]), graph.nodes[-2][3])
# renk x y dis angle angle_kalinan



#temp = list(map(float, input("Hedef: ").split()))
#noktaya_git(temp[0], temp[1], temp[2])
#print(temp)
#stop_thread.append(False)
time.sleep(1)
print("siyah bulana kadar")

print('b', odom)
temp = odom
graph.nodes.append(['b', temp])
while True:
    print("search")
    search()
    print("grafa ekle")
    temp = odom
    graph.nodes.append(['r', temp])
    print('r', odom)
    if not stop_thread[-1]:
        break
    #node benzerligi ile node yoksa ekle bir onceki ile suankinin arasi mesafeyi edge yap
    print("turn")
    turn()

graph.nodes.pop()
temp = odom
print('s', temp)
graph.nodes.append(['s', temp])
print(graph.nodes)
t1.join()
s = siyah(graph.nodes)
b = beyaz(graph.nodes)
print(s, b)
exit(1)
#print(s)
#print(graph.weights)
path = dijsktra(graph, to_coordinate(s[1], s[2]), to_coordinate(b[1], b[2]))
for p in path:
    print(to_x_y(p), end=" -> ")
print()
# # image dinle
# odom dinle dinle
# baslangic : onun bos degilse bosaltana kadar don

# duz git yol bitene kadar. gittigin yolu hesapla. rengi, yolun baslangic noktasi, yol uzunlugu, acisini ve kaldigi aciyi node yap ve stackteki tepedeki node ile edge olustur
# once 120 derece sola don
    # saga donerekten yol ara
    # bulursan ic donguden cik aci degerini kaydederek node a
    # bulamazsan stackten pop at ve eski yerine don kaldigin acidan taramaya devam et

# stack bosalana kadar

# dikjstra at siyah ve beyaz arasi
# o yolu dondur dur

# Python3 implementation to build a
# graph using Dictionaries

