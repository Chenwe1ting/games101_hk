##### hk6

---

运行结果：

![image-20230609185201729](C:\Users\ZQF\AppData\Roaming\Typora\typora-user-images\image-20230609185201729.png)

---

![image-20230609185513449](C:\Users\ZQF\AppData\Roaming\Typora\typora-user-images\image-20230609185513449.png)(C:\Users\ZQF\AppData\Roaming\Typora\typora-user-images\image-20230609185121017.png)

---

##### conclusion:

##### 一般结论是：SAH划分树的生成速度比BVH树慢，但随后的渲染速度更快一点。

#### BVH Idea:

1、objects.size() ==1或==2： 单独讨论

2、objects.size()>=3   遍历所有objects，取坐标跨度最大的轴，然后从小到大排序该坐标，二分生成左右包围盒结点，递归地生成BVH树。

实质还是光线与三角形求交，然后确定某处位置的像素值，计算得到渲染之后的结果。

#### SAH Idea:

1、objects.size() ==1或==2： 单独讨论

2、objects.size()==3  遍历所有的objects,取坐标跨度最大的轴，然后取B份，计算左右包围盒结点取在该处的时间开销，取最小情形时的mincostIndex 进行左右包围盒结点的划分，类似递归地构造SAH树。

实质未变。