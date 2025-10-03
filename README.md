# 杭州电子科技大学PHOENIX战队嵌入式仓库

## 该仓库还在建设中，文档等也在进一步完善，有些还没完成的优化请谅解

## 介绍
本仓库为杭州电子科技大学PHOENIX战队嵌入式框架代码仓库，包含了Bsp层和Module层的代码，旨在为战队成员提供一个统一的代码基础，方便大家进行二次开发和功能扩展。

## 使用指南
该指南以keil5开发为例，如果使用clion或其他开发环境，请根据实际情况进行调整即可 
1. 使用cubemx配置并生成代码(keil5选择`MDK-ARM`)，注意要开启`FreeRTOS`才能使用本仓库。

   ![](pic/1.png)

   编译一次提示无错误无警告则工程创建完成

   ![](pic/2.png)

2. 把本地仓库推送到GitHub，这一步过于简单，不再多说
3. 拉取本仓库作为子模块加入(个人推荐的一种方式)
  ```bash
  git submodule add git@github.com:HDU-PHOENIX/phoenix_embedded_base_code.git [目标路径] # 如果目标路径为空，默认使用仓库名作为目录名。
  ```
  此时在目标路径下会生成一个名为`phoenix_embedded_base_code`的目录，里面包含了本仓库的所有内容。(如果填了目标路径则会使用目标路径作为目录名)
  ![](pic/3.png)
  如图所示，到此步应该在工程目录下会有`.git`,`.gitmodules`和`phoenix_embedded_base_code`三个文件夹，`.git`和`.gitmodules`是Git的配置文件，`phoenix_embedded_base_code`是本仓库的内容。  

4. 在使用本仓库前，需要用户自己创建一个名为 `robot_config.h`的文件，其中需要存放一些关于库的设置信息，具体可复制下文按自己的工程进行修改：
   ```C
   #ifndef __ROBOT_CONFIG_H__
   #define __ROBOT_CONFIG_H__

   /* 是否使用FreeRTOS */
   #define USE_FREERTOS

   /* CAN 类型 */
   #define USER_CAN_STANDARD                   // 使用 标准 CAN
   // #define USER_CAN_FD                      // 使用 CAN FD

   /*CAN 过滤器模式选择*/
   #define USER_CAN_FILTER_MASK_MODE           // 使用掩码模式
   // #define USER_CAN_FILTER_LIST_MODE        // 使用列表模式

   /* CAN 总线启用 */
   #define USER_CAN1                           // 使用 CAN1
   #define USER_CAN2                           // 使用 CAN2
   // #define USER_CAN3                        // 使用 CAN3
               
   /* CAN FIFO 选择 */         
   #define USER_CAN1_FIFO_0                    // 使用 CAN1 FIFO0
   // #define USER_CAN1_FIFO_1                 // 使用 CAN1 FIFO1
   // #define USER_CAN2_FIFO_0                 // 使用 CAN2 FIFO0
   #define USER_CAN2_FIFO_1                    // 使用 CAN2 FIFO1
   // #define USER_CAN3_FIFO_0                 // 使用 CAN3 FIFO0
   // #define USER_CAN3_FIFO_1                 // 使用 CAN3 FIFO1

   #ifdef USE_FREERTOS
   #include "FreeRTOS.h"
   #define user_malloc pvPortMalloc
   #define user_free vPortFree
   #else
   #include "stdlib.h"
   #define user_malloc malloc
   #define user_free free
   #endif

   #endif
   ```
   当然也可以使用仓库中提供的html文件进行配置，但是由于目前队里没人会熟练使用html，故可能会存在一些bug，请仔细检查该文件，不然会影响后续一切库里代码的编译以及运行

5. 此时文件就放到工程目录下了，也做好了相关配置工作，接下来按正常工程的操作添加路径即可使用，具体使用方法参考各模块文档   

6. 添加自己编写的内容
    为了防止自己的代码和仓库代码混淆,此时需要新建一个名为`User_File`的目录存放自己的代码。由于本仓库中封装好了Bsp层和Module层，故正常情况下在`User_File`目录下只需要添加`User_Application`目录即可（或者直接把`User_File`目录直接替换为`User_Application`），存放自己的应用代码，如果您需要使用仓库中没有的模块或算法，可以自行添加`User_Module`等目录存放自己的模块代码。**不过这里强烈建议你把本仓库中没有涉及的模块或算法按照编写要求修改后，以PR形式提交到本仓库中，我们十分期待您的贡献**  
7. 代码编写完成后，在项目文件夹下推送到远端仓库，由于`gitmodule`的特性，您的推送不会对仓库代码造成影响，您可以放心使用。
8. 拉取最新仓库代码，如果有更新，使用以下命令拉取最新代码
   ```bash
   git submodule update --remote
   ```
   这条命令会拉取最新的仓库代码到本地仓库中，注意要进行一次`push`操作才能将最新代码推送到远端仓库中。

### 不想使用gitmodule？
1. 工程使用git: 单独拉取本仓库，然后把内容复制粘贴到自己的工程，其余同上。不粘贴能用，就工程路径会很奇怪，一个工程被分成了两个文件夹
2. 工程未使用git: 直接在仓库目录下拉取本仓库 
总结：还是推荐使用gitmodule的方式，毕竟这样可以更好地管理代码版本和更新。

## 使用须知
- 在使用本仓库的代码时，请不要自行添加或修改任何内容，遇到bug或有更好的实现方式，请以PR或lssues的形式提交到本仓库中，我们会尽快处理。
- 在编译包含本仓库代码的工程时，请确保你已按照各使用的模块的文档要求正确配置，并且请确定使用模块的路径被正确添加到工程中
- 请避免在`User_File`中命名与本仓库中相同名字的文件
- 遇到无法解决的问题，可以直接联系仓库管理员邮箱(hewenxuan923@gmail.com)或(2956889047@qq.com)

