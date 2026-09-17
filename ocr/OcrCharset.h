// ocr/OcrCharset.h
//
// PP-OCRv5 识别模型的字符字典（即 inference.yml 里 PostProcess.character_dict）。
//
// CTC 解码索引约定：
//   index 0        -> 空白符（CTC blank），由解码器直接跳过
//   index 1..N     -> character_dict[0..N-1]
//   index N+1      -> 空格 ' '（PaddleOCR CTCLabelDecode 的 use_space_char 追加项）
//
// 本工程只需要识别编号数字（0-9），因此 decodeDigitOnly 只映射数字字符，
// 其余字符统一按“非数字”丢弃，避免把 'O'/'o' 之类的形近字符当成 0。
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ocr {

    class OcrCharset {
    public:
        /// @brief 从文件加载完整字符字典（每行一个字符，UTF-8 无 BOM）。
        /// @return 成功返回 true；文件不存在或为空返回 false。
        bool loadDict(const std::string& dictPath);

        /// @brief 从 inference.yml 中解析 character_dict（兼容未导出 txt 字典的情况）。
        bool loadFromInferenceYml(const std::string& ymlPath);

        /// @brief 字典是否可用（至少含 1 个字符）。
        bool valid() const { return !m_chars.empty(); }

        /// @brief 字符表大小（用于校验模型输出维度 = size + 2）。
        int numClasses() const { return static_cast<int>(m_chars.size()) + 2; }

        /// @brief 取某个 CTC 索引对应的字符（越界返回空串）。
        const std::string& at(int index) const;

        /// @brief 把 CTC 索引映射为数字字符 '0'-'9'。
        /// @return 命中数字返回其 ASCII 码，否则返回 '\0'。
        char toDigit(int index) const;

        /// @brief 字典中数字字符的数量（正常应为 20：10 个半角 + 10 个全角）。
        int digitCount() const { return static_cast<int>(m_indexToDigit.size()); }

        /// @brief 是否已自动修正「首项全角空格被 trim 掉」导致的索引偏移。
        bool indexRepaired() const { return m_indexRepaired; }

        /// @brief 期望的模型输出类别数（字典大小 + 2：blank 与空格）。
        int expectedClasses() const { return numClasses(); }

        /// @brief 打印前若干项，便于启动时自查索引是否对齐。
        std::string describeFirst(int n) const;

    private:
        void buildDigitIndex();
        /// @brief 自愈：字典首项若是普通汉字，说明全角空格被上游 trim 掉了，
        /// 需补回一项，否则所有 CTC 索引整体偏移 1 位、数字全部解错。
        void repairLeadingTrimmedSpace();

        std::vector<std::string> m_chars;                  // character_dict 原序
        std::unordered_map<int, char> m_indexToDigit;      // ctc index -> '0'..'9'
        bool m_indexRepaired = false;
    };

} // namespace ocr
