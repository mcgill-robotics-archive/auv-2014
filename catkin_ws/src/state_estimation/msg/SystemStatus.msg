#Message for a package to use to communicate its status to the statusGui.

# Topic name: Should be the same for all messages from your node, unless your
# node does multiple independent things that need their own statuses
string topic_name
# Status message: Here you say what you want to say e.g. "Shits on fire!"
string status
# Alert level: Either green, yellow, or red
int32 alert_level
int32 ALERT_GREEN=0
int32 ALERT_YELLOW=1
int32 ALERT_RED=2