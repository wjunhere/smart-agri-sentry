std_msgs/Header header
string status          # "ok" | "timeout" | "error" | "parse_error"
string summary         # 综合分析文本
string[] suggestions   # 建议列表
string risk_level      # "low" | "medium" | "high"
string[] focus_areas   # 关注重点
string next_check      # 建议下次检查时间
string raw_text        # 原始响应 (parse_error时使用)
string trigger         # "auto" | "manual"
