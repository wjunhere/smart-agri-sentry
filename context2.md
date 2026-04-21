## 需求澄清2

1.模型放在了models\finetuned_mobilenetv2_int8.tflite，还未转换格式，目前先用这个模型

2.共有 **10 种** 不同的植物病害（或健康状态）分类，分别是

**bacterial_spot** (细菌性斑点病) 

**early_blight** (早疫病) 

**healthy** (健康叶片)

 **late_blight** (晚疫病)

 **leaf_mold** (叶霉病) 

**septoria_leaf_spot** (壳针孢叶斑病) 

**spider_mites_two-spotted_spider_mite** (二斑叶螨) 

**target_spot** (靶斑病)

 **tomato_mosaic_virus** (番茄花叶病毒)  

**tomato_yellow_leaf_curl_virus** (番茄黄化曲叶病毒)

3.stm32协议你可以根据需要修改，符合实际情况就行

4.Vue 直接连 rosbridge_server 订阅 Topic，FastAPI 只负责航点管理、SQLite、视频流代理

5.摄像头型号是IMX219

6.fusion_node第一版先做一个固定规则，后续再慢慢优化

7.nav_node生成占位框架

###注意：一定要生成一份待办事项清单，你已经确定的信息可以补充或修改原来的PROJECT_CONTEXT.md文件