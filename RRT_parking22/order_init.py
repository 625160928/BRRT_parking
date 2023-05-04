def main2():
    N=100
    orders=[]
    order_list=[[0,1]]
    point_header=0
    point_tail=1
    while point_header<point_tail:
        head=order_list[point_header][0]
        tail=order_list[point_header][1]
        half=(head+tail)/2
        orders.append(half)
        if len(orders)>N:
            return orders

        order_list.append([head,half])
        order_list.append([half,tail])
        point_tail+=2

        point_header+=1
    return orders

def main():
    rem=1/2000
    orders=[0,1]
    order_list=[]
    order_list.append([0,1])
    header=0
    tail=1
    while header<tail:
        now=order_list[header]
        h=now[0]
        t=now[1]
        half=(h+t)/2
        orders.append(half)
        if abs(t-h)>rem:
            order_list.append([h,half])
            order_list.append([half,t])
            tail+=2

        header+=1
    return orders
    print(orders)
    print(len(orders))
if __name__ == '__main__':
    main()