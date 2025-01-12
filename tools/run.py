"""
run.py
automatically run the route
"""

"""修正坐标误差，百度取点使用 BD-09 坐标系，iOS使用 WGS-09 坐标系，进行转换"""
def bd09Towgs84(position):
    import math
    wgs_p = {}

    x_pi = 3.14159265358979324 * 3000.0 / 180.0
    pi = 3.141592653589793238462643383  # π
    a = 6378245.0  # 长半轴
    ee = 0.00669342162296594323  # 偏心率平方

    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * pi) + 20.0 * math.sin(2.0 * x * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * pi) + 40.0 * math.sin(y / 3.0 * pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * pi) + 320 * math.sin(y * pi / 30.0)) * 2.0 / 3.0
        return ret

    def transform_lon(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * pi) + 20.0 * math.sin(2.0 * x * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * pi) + 40.0 * math.sin(x / 3.0 * pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * pi) + 300.0 * math.sin(x / 30.0 * pi)) * 2.0 / 3.0
        return ret

    x = position['lng'] - 0.0065
    y = position['lat'] - 0.006
    z = math.sqrt(x * x + y * y) - 0.00002 * math.sin(y * x_pi)
    theta = math.atan2(y, x) - 0.000003 * math.cos(x * x_pi)

    gcj_lng = z * math.cos(theta)
    gcj_lat = z * math.sin(theta)

    d_lat = transform_lat(gcj_lng - 105.0, gcj_lat - 35.0)
    d_lng = transform_lon(gcj_lng - 105.0, gcj_lat - 35.0)

    rad_lat = gcj_lat / 180.0 * pi
    magic = math.sin(rad_lat)
    magic = 1 - ee * magic * magic
    sqrt_magic = math.sqrt(magic)

    d_lng = (d_lng * 180.0) / (a / sqrt_magic * math.cos(rad_lat) * pi)
    d_lat = (d_lat * 180.0) / (a * (1 - ee) / (magic * sqrt_magic) * pi)

    wgs_p["lat"] = gcj_lat * 2 - gcj_lat - d_lat
    wgs_p["lng"] = gcj_lng * 2 - gcj_lng - d_lng
    return wgs_p

# get the ditance according to the latitude and longitude
def geodistance(p1, p2):
    from geopy.distance import geodesic
    return geodesic((p1["lat"],p1["lng"]),(p2["lat"],p2["lng"])).m

def smooth(start, end, i):
    import math
    i = (i-start)/(end-start)*math.pi
    return math.sin(i)**2

def randLoc(loc: list, v=3, d=0.000025, n=5):
    import random
    import time
    import math
    from statistics import mean, stdev
    
    result = []
    for i in loc:
        result.append(i.copy())

    # 计算中心点
    center = {"lat": 0, "lng": 0}
    for i in result:
        center["lat"] += i["lat"]
        center["lng"] += i["lng"]
    center["lat"] /= len(result)
    center["lng"] /= len(result)
    
    # 根据速度调整基础偏移量
    speed_factor = min(v/3, 2)  # 速度越快偏移越大
    d = d * speed_factor
    
    # 设置随机种子
    random.seed(time.time())
    
    for i in range(n):
        start = int(i*len(result)/n)
        end = int((i+1)*len(result)/n)
        
        # 使用正态分布生成偏移量
        offset = random.gauss(0, d)
        
        for j in range(start, end):
            distance = math.sqrt(
                (result[j]["lat"]-center["lat"])**2 + 
                (result[j]["lng"]-center["lng"])**2
            )
            if 0 == distance:
                continue
            
            # 增强转弯处理
            turn_factor = 1.0
            if j > 0 and j < len(result)-1:
                dx1 = result[j]["lng"] - result[j-1]["lng"]
                dy1 = result[j]["lat"] - result[j-1]["lat"]
                dx2 = result[j+1]["lng"] - result[j]["lng"]
                dy2 = result[j+1]["lat"] - result[j]["lat"]
                angle = abs(math.atan2(dy2, dx2) - math.atan2(dy1, dx1))
                
                # 根据转弯角度动态调整
                turn_factor = 1.0 + math.sin(angle) * (0.3 + random.random() * 0.4)
                
                # 转弯处速度影响
                turn_factor *= speed_factor
            
            # 添加微小随机噪声
            noise = random.gauss(0, d/10)
            
            # 应用偏移
            result[j]["lat"] += ((result[j]["lat"]-center["lat"])/distance * 
                               offset * smooth(start, end, j) * turn_factor + noise)
            result[j]["lng"] += ((result[j]["lng"]-center["lng"])/distance * 
                               offset * smooth(start, end, j) * turn_factor + noise)
    
    # 处理最后一段...与原代码相同
    return result

def fixLockT(loc: list, v, dt):
    fixedLoc = []
    t = 0
    T = []
    T.append(geodistance(loc[1],loc[0])/v)
    a = loc[0].copy()
    b = loc[1].copy()
    j = 0
    while t < T[0]:
        xa = a["lat"] + j*(b["lat"]-a["lat"])/(max(1, int(T[0]/dt)))
        xb = a["lng"] + j*(b["lng"]-a["lng"])/(max(1, int(T[0]/dt)))
        fixedLoc.append({"lat": xa, "lng": xb})
        j += 1
        t += dt
    for i in range(1, len(loc)):
        T.append(geodistance(loc[(i+1)%len(loc)],loc[i])/v + T[-1])
        a = loc[i].copy()
        b = loc[(i+1)%len(loc)].copy()
        j = 0
        while t < T[i]:
            xa = a["lat"] + j*(b["lat"]-a["lat"])/(max(1, int((T[i]-T[i-1])/dt)))
            xb = a["lng"] + j*(b["lng"]-a["lng"])/(max(1, int((T[i]-T[i-1])/dt)))
            fixedLoc.append({"lat": xa, "lng": xb})
            j += 1
            t += dt
    return fixedLoc

def run1(loc: list, v, dt=0.2, speed_variation=0.3):
    import time
    import tools.utils as utils
    import random
    
    # 先计算固定路径点
    fixedLoc = fixLockT(loc, v, dt)
    
    # 随机路径偏移
    nList = (5, 6, 7, 8, 9)
    n = nList[random.randint(0, len(nList)-1)]
    fixedLoc = randLoc(fixedLoc, v,d=0.000025, n=5 )
    
    # 跑步过程中随机调整速度
    clock = time.time()
    for i in fixedLoc:
        # 在每个点随机调整等待时间,相当于调整速度
        dt_random = dt * (1 + (2*random.random()-1) * speed_variation)
        utils.setLoc(bd09Towgs84(i))
        while time.time()-clock < dt_random:
            pass
        clock = time.time()

def run(loc: list, v, d=45):
    import random
    import time
    random.seed(time.time())
    while True:
        vRand = 1000/(1000/v-(2*random.random()-1)*d)
        run1(loc, vRand)
        print("跑完一圈了")