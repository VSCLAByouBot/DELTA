# 台達機械手臂控制程式
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]

## 編譯環境

* **作業系統:** Windows 10

* **IDE:** Visual Studio 2017

## 控制訊號硬體配置

* [工研院 IMP-3 智慧型運動控制平台](https://www.epcio.com.tw/product/IMP-3.aspx)
* [工研院 IMP-ARIO 非同步遠端I/O控制子板](https://www.epcio.com.tw/product/IMP-ARIO.aspx)

## 程式專案描述

### `00_ClearCmd`

清除軸卡命令

### `01_ReadEncoder`

讀取各軸之馬達編碼器角度

### `02_PTPGenerator`

產生點對點運動之軌跡命令檔

### `03_PTPControl`

對機械手臂進行點對點運動控制

### `04_Tracking`

讓機械手臂根據輸入軌跡命令檔來運動

## Gadgets

* `clean_Solution.bat`
雙擊執行後，可以自動清理不必要的VS設定檔、編譯檔、記錄檔等

## Dependencies

### `IMC3Driver.lib`

#### Installation

請下載並安裝[工研院](https://www.epcio.com.tw/support/download.aspx)所提供的軸卡驅動程式 [IMP_Series_V.3.03.zip](https://www.epcio.com.tw/support/download/IMP_Series_V.3.03.zip)

#### Apply to Visual Studio Project

Visual Studio 專案 (Project) 右鍵＞屬性

1. VC++目錄＞程式庫目錄
`C:\Program Files (x86)\ITRI\IMP3\IDDL\Library\VC`
2. 連結器＞輸入＞其他相依性
`IMC3Driver.lib`

## Details

* [VSCLab 共用雲端/04. 機械手臂技術文件/DELTA](https://ncku365-my.sharepoint.com/personal/e14054112_ncku_edu_tw/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fe14054112%5Fncku%5Fedu%5Ftw%2FDocuments%2F04%2E%20%E6%A9%9F%E6%A2%B0%E6%89%8B%E8%87%82%E6%8A%80%E8%A1%93%E6%96%87%E4%BB%B6%2FDELTA&view=0)
  * `基礎操作與緊急停機處置SOP.pdf`
  * `台達手臂新建專案SOP.docx`
