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

