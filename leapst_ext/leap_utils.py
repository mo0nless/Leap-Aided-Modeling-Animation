#Metodo che raggruppa le mani rilevate nel frame in un dizionario 
def get_hands_dict(frame):
    hands = frame.hands
    D = {"Left":None,"Right":None}
    for hand in hands:
        if hand.is_left:
            D["Left"] = hand
        if hand.is_right:
            D["Right"] = hand
    return D

#Metodo ricorsivo che calcola il numero di pixel luminosi nell'immagine img_data attorno a quello passato in input
def number_of_marked_pixels(img_data, w, h, PX, PY, S, maxS, checkedSet):
    checkedSet.add(w*PY + PX)
    S += 1
    maxX,maxY = PX,PY
    minX,minY = PX,PY
    if S >= maxS:
        return S,minX,minY,maxX,maxY,checkedSet
    if PX > 0 and (w*PY + PX - 1) not in checkedSet and img_data[w*PY + PX - 1] > 250:
        S,sminx,sminy,smaxx,smaxy,checkedSet = number_of_marked_pixels(img_data,w,h,PX-1,PY,S,maxS,checkedSet)
        minX = min(sminx,minX)
        maxX = max(smaxx,maxX)
        minY = min(sminy,minY)
        maxY = max(smaxy,maxY)
        if S >= maxS:
            return S,minX,minY,maxX,maxY,checkedSet
    if PX < w and (w*PY + PX + 1) not in checkedSet and img_data[w*PY + PX + 1] > 250:
        S,sminx,sminy,smaxx,smaxy,checkedSet = number_of_marked_pixels(img_data,w,h,PX+1,PY,S,maxS,checkedSet)
        minX = min(sminx,minX)
        maxX = max(smaxx,maxX)
        minY = min(sminy,minY)
        maxY = max(smaxy,maxY)
        if S >= maxS:
            return S,minX,minY,maxX,maxY,checkedSet
    if PY > 0 and (w*(PY-1) + PX) not in checkedSet and img_data[w*(PY-1) + PX] > 250:
        S,sminx,sminy,smaxx,smaxy,checkedSet = number_of_marked_pixels(img_data,w,h,PX,PY-1,S,maxS,checkedSet)
        minX = min(sminx,minX)
        maxX = max(smaxx,maxX)
        minY = min(sminy,minY)
        maxY = max(smaxy,maxY)
        if S >= maxS:
            return S,minX,minY,maxX,maxY,checkedSet
    if PY < h and (w*(PY+1) + PX) not in checkedSet and img_data[w*(PY+1) + PX] > 250:
        S,sminx,sminy,smaxx,smaxy, checkedSet = number_of_marked_pixels(img_data,w,h,PX,PY+1,S,maxS,checkedSet)
        minX = min(sminx,minX)
        maxX = max(smaxx,maxX)
        minY = min(sminy,minY)
        maxY = max(smaxy,maxY)     
    return S,minX,minY,maxX,maxY,checkedSet
