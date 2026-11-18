# smallcar — v1

小车代码备份，整理于 2026-11-18。**这是当前工作归档，不表示已解决全部巡航、定向和回充故障。没有因本次提交部署程序或发车。**

| 目录 | 内容 |
|---|---|
| [deployments/1003/demo](deployments/1003/demo) | 1003最近实车诊断快照中的实际导航、网页与控制代码 |
| [windows](windows) | 电脑连上Wi-Fi后的检测、SSH启动和打开网页脚本 |
| [experiments](experiments/README.md) | 未部署的朝向修改、未启用的深度避障、暂停的RSSI回传 |
| 根目录demo/tools/rrt_mapping/ws63/vendor | 上游9d67bd1部署基线与依赖源码，和1003快照分开保存 |

实际地图、桩位、巡航点、录包、密码及密钥不上传到公开仓库。附带地图仅为生成的示例。**不要把根目录或实验目录直接覆盖到正在使用的小车上。**

- [v1状态、来源和恢复边界](docs/v1-status.md)
- [与上游最新版的功能及冲突对比](docs/upstream-comparison-20261118.md)
- [上游基线使用说明](README-upstream-baseline.md)
- [源码清单](docs/current-source-manifest.json)

上游来源为 [Lyr1cs13/r550-field-kit](https://github.com/Lyr1cs13/r550-field-kit)，保留原MIT许可和第三方许可。`v1`版本分支用于标识本次归档。

