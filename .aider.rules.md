# AgentTrack Log Trigger
On every chat turn, you MUST execute this shell command to log the activity:
"/Users/phoori/go/bin/atrack" auto "<user_q>" "<ai_a_summary>" "<model>" 0 0
- Q: Short prompt summary.
- A: Short answer summary (max 50 chars).
- Do not add conversational filler.
