## 1.1 了解Git

Git是一个免费的、开源的分布式版本控制系统，可以高速处理从小型到大型的各种项目
版本控制：是一种记录文件内容变化，以便将来查阅特定版本修订情况的系统
了解一下：集中式与分布式版本控制工具

         -- 集中式版本控制工具：如CVS、`SVN`等，都有一个单一的几种管理服务器，保存所有文件的修订版本，而协同工作的人通过客户端连接到这台服务器，从而取出最新的文件或者提交更新。缺点：中央服务器的单点故障；多(程序员)对一(中央服务器)
    
         -- 分布式版本控制工具：如git,客户端取的不是最新的文件快照，而是把代码仓库完整的镜像下来到本地库(克隆/备份)

工作机制：

![img](https://nack-1316646329.cos.ap-nanjing.myqcloud.com/69133d076d5567079e4d01b8be137e83.png)

## 1.2 Git安装

略

## 2.1 常用命令

| 命令                                                      | 说明                             |
| :-------------------------------------------------------- | -------------------------------- |
| git config --global user.name 用户名                      | 设置用户签名                     |
| git config --global user.email 邮箱                       | 设置用户签名                     |
| git init                                                  | 初始化本地库                     |
| git status                                                | 查看本地库状态                   |
| git add 文件名                                            | 添加到暂存区                     |
| git commit-m "日志信息" 文件名                            | 提交到本地库                     |
| git reflog/git log                                        | 查看历史记录                     |
| git reset --hard 版本号                                   | 版本穿梭（谨慎使用）             |
| git remote add origin git@github.com:author/myProject.git | 添加到远程服务器                 |
| git push -u origin master                                 | 上传到master分支                 |
| git pull origin master                                    | 拉去master分支修改更新本地代码   |
| git clone git@github.com:author/myProject.git             | 从远程仓库克隆Project Code到本地 |

## 2.2 基本操作

略

## 2.3.1 分支的好处

-  同时并进行多个功能开发，提高了开发效率
- 各个分支再开发过程中，如果某个分支开发失败，不会对其他分支有任何影响，失败的分支删除重新开始即可

## 2.3.2 分支操作常用命令

| 命令                       | 说明                         |
| -------------------------- | ---------------------------- |
| git branch 分支名          | 创建分支                     |
| git branch -v              | 查看分支                     |
| git checkout 分支名        | 切换分支                     |
| git merge 需要合并的分支名 | 把指定的分支合并到当前分支上 |

## 总结

| 命令                         | 功能描述                                       | 示例                                 |
| ---------------------------- | ---------------------------------------------- | ------------------------------------ |
| 基础命令                     |                                                |                                      |
| git init                     | 初始化一个新的本地 Git 仓库                    | git init                             |
| git clone <url>              | 从远程仓库克隆代码                             | git clone                            |
| git status                   | 查看工作区和暂存区状态                         | git status                           |
| git log                      | 查看提交历史                                   | git log                              |
| git config                   | 配置用户名和邮箱地址                           | git config --global user.name "Name" |
| 分支管理                     |                                                |                                      |
| git branch                   | 查看本地分支                                   | git branch                           |
| git branch <branch-name>     | 创建新分支                                     | git branch feature                   |
| git checkout <branch-name>   | 切换分支                                       | git checkout main                    |
| git switch <branch-name>     | 切换分支（推荐使用）                           | git switch main                      |
| git merge <branch-name>      | 合并分支                                       | git merge feature                    |
| git branch -d <branch-name>  | 删除分支                                       | git branch -d feature                |
| 添加和提交                   |                                                |                                      |
| git add <file>               | 将文件添加到暂存区                             | git add file.txt                     |
| git add .                    | 添加当前目录的所有更改文件                     | git add .                            |
| git commit -m "message"      | 提交暂存区更改到本地仓库                       | git commit -m "Add new feature"      |
| git commit --amend           | 修改最后一次提交信息                           | git commit --amend                   |
| 远程操作                     |                                                |                                      |
| git remote -v                | 查看远程仓库信息                               | git remote -v                        |
| git remote add <name> <url>  | 添加远程仓库                                   | git remote add origin <url>          |
| git pull <remote> <branch>   | 从远程仓库拉取最新代码                         | git pull origin main                 |
| git push <remote> <branch>   | 推送本地分支代码到远程仓库                     | git push origin main                 |
| 撤销和恢复                   |                                                |                                      |
| git checkout -- <file>       | 撤销工作区的更改（回到最近一次暂存或提交状态） | git checkout -- file.txt             |
| git reset <file>             | 取消文件的暂存（从暂存区移回工作区）           | git reset file.txt                   |
| git reset --hard             | 重置工作区和分支到指定提交                     | git reset --hard HEAD~1              |
| 查看和对比                   |                                                |                                      |
| git diff                     | 查看工作区未暂存的更改                         | git diff                             |
| git diff --cached            | 查看暂存区和上一次提交的差异                   | git diff --cached                    |
| git diff <branch1> <branch2> | 查看两个分支之间的差异                         | git diff main feature                |
| 标签管理                     |                                                |                                      |
| git tag                      | 查看所有标签                                   | git tag                              |
| git tag <tag-name>           | 创建一个标签                                   | git tag v1.0                         |
| git tag -d <tag-name>        | 删除一个标签                                   | git tag -d v1.0                      |
| git push origin <tag-name>   | 推送标签到远程仓库                             | git push origin v1.0                 |



## SOP

```
step1：
从github创建仓库后使用git clone 保存至本地 （这样就不用费心链接了）
step2：
git add +文件名 / git add . 上传全部
step3：
git commit -m "注释"
step3：
git push -u origin main
step4:
git pull
```

## Notice

![image-20250301103920429](https://nack-1316646329.cos.ap-nanjing.myqcloud.com/image-20250301103920429.png)

```
git rm --cached <File Name>
git add . 
git commit -m "delete .git"
git push
```

