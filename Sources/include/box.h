#include <stdio.h>
#include <wchar.h>
#include <locale.h>
#include <vector>
#include <string>
#include <sstream>

enum class ELang
{
    TW,
    ENG
};

// Choose your language preference
ELang hint_lang_option = ELang::TW;

const int BOX_WIDTH = 70;

// Box Char ASCII code
const wchar_t box_lu = 0x2554;
const wchar_t box_ru = 0x2557;
const wchar_t box_ld = 0x255A;
const wchar_t box_rd = 0x255D;
const wchar_t box_hh = 0x2550;
const wchar_t box_vv = 0x2551;
const wchar_t box_vr = 0x2560;
const wchar_t box_vl = 0x2563;
const wchar_t box_hd = 0x2566;
const wchar_t box_hu = 0x2569;
const wchar_t box_hv = 0x256C;
const wchar_t left_box[] = {box_lu, box_ld, box_vr};
const wchar_t right_box[] = {box_ru, box_rd, box_vl};

std::vector<std::vector<std::string>> hint_ENG{
    {"Program Description"},                                                // 程式名稱與簡述
    {"WARNING! SAFETY CHECKLIST:",                                          // 警告! 請執行以下安全檢查事項:
	 "1. Make sure the EMERGENCY and ERROR BUTTON are disabled.",			// 確認機台緊急開關跳脫,軸卡錯誤燈號熄滅
     "2. Make sure the workspace around the robot is clean.",               // 確認機台周遭已淨空
     "3. Make sure one is ready to press 'Esc' and EMERGENCY BUTTON.",      // 操作員務必一手置於ESC按鍵上,另一手置於緊急開關上
     "4. Make sure the computer is not under heavy load.",					// 確認電腦負荷不會過重
     "5. Press 'Esc' to close the program.",                                // 無論如何,務必按ESC關閉程式,
     "   If not, clear MC command first before running the next program!"}, // 若意外中斷,請先清軸卡命令,再繼續做實驗
    {"HINT:",                                                               // 操作提示:
     "1. Press any key to start.",                                          // 按任意鍵開始
     "2. Press 'Esc' to end program anytime."}};                            // 隨時按ESC關閉程式

std::vector<std::vector<std::string>> hint_TW{
    {"程式名稱與簡述"},
    {"警告! 請執行以下安全檢查事項:",
	 "1. 確認機台緊急開關跳脫,軸卡錯誤燈號熄滅",
     "2. 確認機台周遭已淨空",
     "3. 操作員務必一手置於ESC按鍵上,另一手置於緊急開關上",
     "4. 確認電腦負荷不會過重",
     "5. 無論如何,務必按ESC關閉程式,",
     "   若意外中斷,請先清軸卡命令,再繼續做實驗"},
    {"操作提示:",
     "1. 按任意鍵開始",
     "2. 隨時按ESC關閉程式"}};

/**
 * @brief 在文字兩側填滿指定寬度的符號
 * @param str: 輸入文字
 * @param c: 填滿文字兩側的符號
 * @param width: 總寬度
 */
void line(const std::string &str, char c, size_t width)
{
    size_t space_num_1 = (width - str.size()) / 2;
    size_t space_num_2 = width - str.size() - space_num_1;
    std::string line1(space_num_1, c), line2(space_num_2, c);
    printf("%s%s%s", line1.c_str(), str.c_str(), line2.c_str());
}

/**
 * @brief 在文字兩側填滿指定寬度的符號,並換行
 * @param state: 輸入文字
 * @param c: 填滿文字兩側的符號
 */
void put_line(const std::string &state, char c = '=')
{
    line(state, c, BOX_WIDTH + 2);
    printf("\n");
}

/**
 * @brief 顯示方框的邊界
 * @param box_type: 邊界形式(0: 上, 1: 下, 2: 中)
 */
void box_edge(int box_type)
{
    wprintf(L"%lc", left_box[box_type]);
    for (int i = 0; i < BOX_WIDTH; ++i)
        wprintf(L"%lc", box_hh);
    wprintf(L"%lc\n", right_box[box_type]);
}

/**
 * @brief 顯示方框內的多行文字
 * @param hint: 提示文字,vector中的每個元素代表一行字
 * @param middle: 是否要置中對齊
 */
void box_content(std::vector<std::string> &hints, bool middle = false)
{
    for (auto &hint : hints)
    {
        if (hint.size() > BOX_WIDTH)
        {
            puts("box_content(): Error! Too much text.");
            return;
        }
        wprintf(L"%lc", box_vv); // 左側垂直線
        if (middle)
        {
            line(hint, ' ', BOX_WIDTH);
        }
        else
        {
            int space_num = BOX_WIDTH - 1 - (int)hint.size();
            printf(" %s%*s", hint.c_str(), space_num, "");
        }
        wprintf(L"%lc\n", box_vv); // 右側垂直線
    }
}

/**
 * @brief 顯示方框內的多行文字
 * @param messages: 提示文字,第一維代表一個方框,第二維代表一行字
 * @param middle: 是否要置中對齊
 */
void message_box(std::vector<std::vector<std::string>> &messages)
{
    setlocale(LC_CTYPE, "");
    box_edge(0);
    for (int i = 0; i < messages.size(); ++i)
    {
        box_content(messages[i], i == 0); // middle alignment for first line
        if (i < messages.size() - 1)
            box_edge(2);
    }
    box_edge(1);
}

/**
 * @brief 字串分割
 * @ref https://shengyu7697.github.io/cpp-string-split/
 * @param str: 輸入字串
 * @param pattern: 分割字元
 * @return 分割好的字串
 */
const std::vector<std::string> split(const std::string &str, const std::string &pattern)
{
    std::vector<std::string> result;
    std::string::size_type begin, end;

    end = str.find(pattern);
    begin = 0;

    while (end != std::string::npos)
    {
        if (end - begin != 0)
        {
            result.push_back(str.substr(begin, end - begin));
        }
        begin = end + pattern.size();
        end = str.find(pattern, begin);
    }

    if (begin != str.length())
    {
        result.push_back(str.substr(begin));
    }
    return result;
}

/**
 * @brief 顯示程式概述與操作提示(以方框顯示)
 * @param program_info: 自訂程式概述字串
 */
void put_hint(std::string program_info)
{
    if (hint_lang_option == ELang::ENG)
    {
        hint_ENG[0] = split(program_info, "\n");
        message_box(hint_ENG);
    }
    else
    {
        hint_TW[0] = split(program_info, "\n");
        message_box(hint_TW);
    }
}
