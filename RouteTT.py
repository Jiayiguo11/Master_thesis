import traci


def routetraveltime(roid):
    routesets = []
    routesets.append(("em3_4", "em4_5", "em5_6", "em6_7", "em7_8", "em8_9", "em9_10", "em10_11", "em11_12", "em12_13", "e13_14", "e14_15", "e15_16"))
    routesets.append(("offramp1_1", "offramp1_2", "offramp1_3", "onramp3", "es5_6", "es6_7", "es7_8", "ns8_nm13", "e13_14", "e14_15", "e15_16"))

    rs = routesets[roid]
    tt = 0
    for edge in rs:
        tt += traci.edge.getTraveltime(edge)


    return tt