// ocr/OcrCharset.cpp
#include "ocr/OcrCharset.h"

#include <fstream>
#include <sstream>
#include <algorithm>

namespace ocr {
    namespace {

        /// @brief 把一个 UTF-8 字符串（应为单字符）转成数字，不是数字则返回 '\0'。
        /// 支持半角 '0'-'9' 与全角 '０'-'９'。
        char utf8ToDigit(const std::string& s) {
            // 半角数字
            if (s.size() == 1 && s[0] >= '0' && s[0] <= '9') {
                return s[0];
            }
            // 全角数字 U+FF10..U+FF19 : EF BC 90 .. EF BC 99
            if (s.size() == 3 &&
                static_cast<unsigned char>(s[0]) == 0xEF &&
                static_cast<unsigned char>(s[1]) == 0xBC) {
                unsigned char b2 = static_cast<unsigned char>(s[2]);
                if (b2 >= 0x90 && b2 <= 0x99) {
                    return static_cast<char>('0' + (b2 - 0x90));
                }
            }
            return '\0';
        }

        /// @brief 去掉行尾的 \r 与首尾空白。
        std::string trim(const std::string& in) {
            size_t b = in.find_first_not_of(" \t\r\n");
            if (b == std::string::npos) return {};
            size_t e = in.find_last_not_of(" \t\r\n");
            return in.substr(b, e - b + 1);
        }

        /// @brief 解析 YAML 列表项的值部分，处理引号与 '' 转义。
        /// 例如  `- '0'` -> `0`   `- ''''` -> `'`   `- ％` -> `％`
        std::string parseYamlScalar(const std::string& raw) {
            std::string v = trim(raw);
            if (v.size() >= 2 && v.front() == '\'' && v.back() == '\'') {
                std::string inner = v.substr(1, v.size() - 2);
                // YAML 单引号内的 '' 表示一个单引号
                std::string out;
                out.reserve(inner.size());
                for (size_t i = 0; i < inner.size(); ++i) {
                    if (inner[i] == '\'' && i + 1 < inner.size() && inner[i + 1] == '\'') {
                        out.push_back('\'');
                        ++i;
                    }
                    else {
                        out.push_back(inner[i]);
                    }
                }
                return out;
            }
            if (v.size() >= 2 && v.front() == '"' && v.back() == '"') {
                return v.substr(1, v.size() - 2);
            }
            return v;
        }

    } // namespace

    bool OcrCharset::loadDict(const std::string& dictPath) {
        std::ifstream in(dictPath, std::ios::binary);
        if (!in.is_open()) {
            return false;
        }
        // 跳过 UTF-8 BOM
        if (in.peek() == 0xEF) {
            char bom[3];
            in.read(bom, 3);
            if (!(static_cast<unsigned char>(bom[0]) == 0xEF &&
                  static_cast<unsigned char>(bom[1]) == 0xBB &&
                  static_cast<unsigned char>(bom[2]) == 0xBF)) {
                in.seekg(0);
            }
        }
        m_chars.clear();
        std::string line;
        while (std::getline(in, line)) {
            // 只去掉行结束符：PP-OCRv5 字典第 0 项是「全角空格」，不能被 trim 掉，
            // 否则其后所有字符的 CTC 索引都会整体偏移 1 位。
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            if (!line.empty()) {
                m_chars.push_back(line);
            }
        }
        repairLeadingTrimmedSpace();
        buildDigitIndex();
        return !m_chars.empty();
    }

    bool OcrCharset::loadFromInferenceYml(const std::string& ymlPath) {
        std::ifstream in(ymlPath, std::ios::binary);
        if (!in.is_open()) {
            return false;
        }
        m_chars.clear();
        std::string line;
        bool inDict = false;
        while (std::getline(in, line)) {
            std::string t = trim(line);
            if (!inDict) {
                // 定位 `character_dict:` 起始
                if (t.rfind("character_dict:", 0) == 0) {
                    inDict = true;
                }
                continue;
            }
            // 列表项形如 "- X"
            if (t.size() >= 2 && t[0] == '-' && (t[1] == ' ' || t[1] == '\t')) {
                std::string val = parseYamlScalar(t.substr(2));
                if (!val.empty()) {
                    m_chars.push_back(val);
                }
                continue;
            }
            if (t == "-") {
                // 空项，跳过
                continue;
            }
            // 遇到非列表内容说明字典结束
            break;
        }
        repairLeadingTrimmedSpace();
        buildDigitIndex();
        return !m_chars.empty();
    }

    void OcrCharset::repairLeadingTrimmedSpace() {
        m_indexRepaired = false;
        if (m_chars.empty()) {
            return;
        }
        // 正常的 PP-OCRv5 字典第 0 项是全角空格 U+3000 (E3 80 80)。
        // 若上游用 trim() 处理过（例如 PowerShell 的 .Trim()），该项会被删掉，
        // 于是第 0 项变成一个普通汉字，整体索引偏移 1。
        static const std::string kIdeographicSpace = "\xE3\x80\x80";
        if (m_chars.front() == kIdeographicSpace) {
            return;
        }
        // 仅当第 0 项确实像「本该是空格的位置被汉字顶替」时才补：
        // 该项必须是 CJK 汉字（UTF-8 三字节且落在常见汉字区），否则不动。
        const std::string& first = m_chars.front();
        if (first.size() != 3) {
            return; // ASCII 或其它，不做猜测
        }
        const unsigned char b0 = static_cast<unsigned char>(first[0]);
        if (b0 < 0xE4 || b0 > 0xE9) {
            return; // 不是 CJK 统一汉字起始区
        }
        m_chars.insert(m_chars.begin(), kIdeographicSpace);
        m_indexRepaired = true;
    }

    void OcrCharset::buildDigitIndex() {
        m_indexToDigit.clear();
        for (size_t i = 0; i < m_chars.size(); ++i) {
            char d = utf8ToDigit(m_chars[i]);
            if (d != '\0') {
                // CTC 索引 = 字典下标 + 1（0 被空白符占用）
                m_indexToDigit[static_cast<int>(i) + 1] = d;
            }
        }
    }

    const std::string& OcrCharset::at(int index) const {
        static const std::string kEmpty;
        if (index <= 0 || index > static_cast<int>(m_chars.size())) {
            return kEmpty;
        }
        return m_chars[static_cast<size_t>(index) - 1];
    }

    char OcrCharset::toDigit(int index) const {
        auto it = m_indexToDigit.find(index);
        return it == m_indexToDigit.end() ? '\0' : it->second;
    }

    std::string OcrCharset::describeFirst(int n) const {
        std::ostringstream oss;
        oss << "charset=" << m_chars.size() << " digits=" << m_indexToDigit.size() << " head=[";
        int limit = std::min<int>(n, static_cast<int>(m_chars.size()));
        for (int i = 0; i < limit; ++i) {
            if (i) oss << ",";
            oss << (i + 1) << ":" << m_chars[static_cast<size_t>(i)];
        }
        oss << "]";
        return oss.str();
    }

} // namespace ocr
