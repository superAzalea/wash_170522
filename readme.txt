170522
主要问题:
1 为取衣功能增加长按键。
3 手动运行：
按洗涤，运行，stop再进入会一直发送 01 10 00 02 00 03 06 00 50 00 14 02 58 C7 D8，得不到回应。
2 
(1)P2[3]取衣功能,
(2) P3[2]->P3[6]自动补热温差,P3[5]自动运行是否可以手动;
(3) P10  
	单向结束延时   需去掉 
        均布结束延时       
                 
        均步结束刹车          需修改
P3[5]自动运行是否可以手动;

4 发停止音5分钟
本工程解决的问题：
1 取衣功能设置完成：当长按取衣键是，开始取衣，长按结束，取衣结束。