# PHOENIX嵌入式框架代码开发指南

## 开发流程
### 方式一：直接在原仓库拉分支
#### 1. 拉取远端仓库
在本地文件夹下打开终端，输入
``` bash
git clone git@github.com:HDU-PHOENIX/phoenix_embedded_base_code.git
```
拉取仓库代码到本地
#### 2. 创建自己的开发分支
输入
``` bash
git checkout -b <分支名>
```
或者
``` bash
git switch -c <分支名>
```
创建并切换到新分支  
分支名规范：
| 分支名前缀 | 分支类型 | 使用示例 |
| :---: | :---: | :---: |
| feature | 新功能 | feature/can |
| fix | 修复bug | fix/can |
| hotfix | 紧急修复 | hotfix/can |
| release | 版本发布 | release/v1.0 |
> hotfix：通常指的是针对线上（生产环境）出现的紧急问题进行的快速修复。它是从主分支（master）拉取的修复分支，修复完成后需要合并回主分支和开发分支，以确保问题得到及时解决。  
fix：一般指的是在开发过程中对代码的常规修复，可能不涉及紧急情况。它可以是任何类型的错误修复，通常在下一个版本发布中进行。

可以使用
``` bash
git branch
```
查看当前分支，如果新建并切换分支成功，会看到新分支前有星号标记
#### 3. 开发代码
在分支切换成功后，就可以在本地进行代码开发了。完成代码开发后，使用
``` bash
git add .
git commit -m "提交信息"
git push origin <分支名>
```
把代码推送到远端分支
**(注意不要推送到main分支！！！！！)**\
commit信息规范：
| 规范 | 示例 |
| :---: | :---: |
| feat | feat(can): add new feature |
| fix | fix(can): resolve issue |
| docs | docs(can): update documentation |
| style | style(can): format code |
| refactor | refactor(can): improve code structure |
| test | test(can): add unit tests |
可以使用vscode的git-commit-plugin等插件来规范提交代码
#### 4. 提交合并请求
当一个功能开发完成，且完成测试后在github上打开仓库页面，点击"Pull requests"选项卡，然后点击"New pull request"按钮，选择要合并的分支，填写合并请求信息，最后点击"Create pull request"按钮提交合并请求。
#### 5. 等待合并
合并请求提交后，等待其他开发者审核和合并代码。如果管理员拒绝了pr，请根据反馈进行修改后重新提交。如果提交的pr被批准后，管理员会将代码合并到主分支（main）中，并删除远端的开发分支，下一次开发时需要再次新建远端分支(本地分支不受影响)
### 方式二：Fork仓库
#### 1. Fork仓库
在github上打开仓库页面，点击右上角的"Fork"按钮，将仓库复制到自己的账号下。
#### 2. 克隆仓库到本地
在本地文件夹下打开终端，输入
``` bash
git clone git@github.com:<你的用户名>/phoenix_embedded_base_code.git
```
将Fork后的仓库克隆到本地。
#### 3. 开发代码
可以直接在克隆的仓库的main分支进行开发，也可以创建自己的开发分支，方法同上。
然后提交代码到远端克隆仓库
#### 4，5. 提交PR同上
