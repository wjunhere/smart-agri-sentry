std_msgs/Header header
float32 risk_score
uint8 alert_level
string mode
string[] evidence_chain
float32 lwd_hours
float32 confidence
# Weighted contribution of each input branch to risk_score;
# the three terms sum to risk_score (for UI breakdown display).
float32 vision_term
float32 env_term
float32 interaction_term
