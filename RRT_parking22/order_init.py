

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
    print(orders)
    print(len(orders))
if __name__ == '__main__':
    main()