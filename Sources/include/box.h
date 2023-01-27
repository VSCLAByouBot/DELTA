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
    {"Program Description"},                                                // �{���W�ٻP²�z
    {"WARNING! SAFETY CHECKLIST:",                                          // ĵ�i! �а���H�U�w���ˬd�ƶ�:
	 "1. Make sure the EMERGENCY and ERROR BUTTON are disabled.",			// �T�{���x���}������,�b�d���~�O������
     "2. Make sure the workspace around the robot is clean.",               // �T�{���x�P�D�w�b��
     "3. Make sure one is ready to press 'Esc' and EMERGENCY BUTTON.",      // �ާ@���ȥ��@��m��ESC����W,�t�@��m����}���W
     "4. Make sure the computer is not under heavy load.",					// �T�{�q���t�����|�L��
     "5. Press 'Esc' to close the program.",                                // �L�צp��,�ȥ���ESC�����{��,
     "   If not, clear MC command first before running the next program!"}, // �Y�N�~���_,�Х��M�b�d�R�O,�A�~�򰵹���
    {"HINT:",                                                               // �ާ@����:
     "1. Press any key to start.",                                          // �����N��}�l
     "2. Press 'Esc' to end program anytime."}};                            // �H�ɫ�ESC�����{��

std::vector<std::vector<std::string>> hint_TW{
    {"�{���W�ٻP²�z"},
    {"ĵ�i! �а���H�U�w���ˬd�ƶ�:",
	 "1. �T�{���x���}������,�b�d���~�O������",
     "2. �T�{���x�P�D�w�b��",
     "3. �ާ@���ȥ��@��m��ESC����W,�t�@��m����}���W",
     "4. �T�{�q���t�����|�L��",
     "5. �L�צp��,�ȥ���ESC�����{��,",
     "   �Y�N�~���_,�Х��M�b�d�R�O,�A�~�򰵹���"},
    {"�ާ@����:",
     "1. �����N��}�l",
     "2. �H�ɫ�ESC�����{��"}};

/**
 * @brief �b��r�ⰼ�񺡫��w�e�ת��Ÿ�
 * @param str: ��J��r
 * @param c: �񺡤�r�ⰼ���Ÿ�
 * @param width: �`�e��
 */
void line(const std::string &str, char c, size_t width)
{
    size_t space_num_1 = (width - str.size()) / 2;
    size_t space_num_2 = width - str.size() - space_num_1;
    std::string line1(space_num_1, c), line2(space_num_2, c);
    printf("%s%s%s", line1.c_str(), str.c_str(), line2.c_str());
}

/**
 * @brief �b��r�ⰼ�񺡫��w�e�ת��Ÿ�,�ô���
 * @param state: ��J��r
 * @param c: �񺡤�r�ⰼ���Ÿ�
 */
void put_line(const std::string &state, char c = '=')
{
    line(state, c, BOX_WIDTH + 2);
    printf("\n");
}

/**
 * @brief ��ܤ�ت����
 * @param box_type: ��ɧΦ�(0: �W, 1: �U, 2: ��)
 */
void box_edge(int box_type)
{
    wprintf(L"%lc", left_box[box_type]);
    for (int i = 0; i < BOX_WIDTH; ++i)
        wprintf(L"%lc", box_hh);
    wprintf(L"%lc\n", right_box[box_type]);
}

/**
 * @brief ��ܤ�ؤ����h���r
 * @param hint: ���ܤ�r,vector�����C�Ӥ����N��@��r
 * @param middle: �O�_�n�m�����
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
        wprintf(L"%lc", box_vv); // ���������u
        if (middle)
        {
            line(hint, ' ', BOX_WIDTH);
        }
        else
        {
            int space_num = BOX_WIDTH - 1 - (int)hint.size();
            printf(" %s%*s", hint.c_str(), space_num, "");
        }
        wprintf(L"%lc\n", box_vv); // �k�������u
    }
}

/**
 * @brief ��ܤ�ؤ����h���r
 * @param messages: ���ܤ�r,�Ĥ@���N��@�Ӥ��,�ĤG���N��@��r
 * @param middle: �O�_�n�m�����
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
 * @brief �r�����
 * @ref https://shengyu7697.github.io/cpp-string-split/
 * @param str: ��J�r��
 * @param pattern: ���Φr��
 * @return ���Φn���r��
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
 * @brief ��ܵ{�����z�P�ާ@����(�H������)
 * @param program_info: �ۭq�{�����z�r��
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
