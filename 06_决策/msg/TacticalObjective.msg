# A candidate approved by a future map/route/observation adapter, not an arbitrary
# target invented by the policy. No map coordinates are supplied by this package.
uint8 RECOVERY=0
uint8 ZONE=1
uint8 DEFENCE=2
uint8 PATROL=3
uint8 GROUND=0
uint8 HIGHLAND=1
uint8 TUNNEL=2
string<=64 id
uint8 kind
uint8 terrain
bool reachable
bool route_verified
bool route_uses_highland
bool route_uses_tunnel
bool risk_known
float64 risk  # Normalized engineering estimate [0,1]; unknown is NOT zero.
float64 travel_time_s
float64 x
float64 y
float64 radius
# Owner uses DecisionState OWNER_* constants, relative to our team.
uint8 owner
bool maintain_required
bool self_present
bool vacancy_confirmed
bool withdrawal_support
bool handover_confirmed
# vacancy_confirmed needs positive fresh evidence, not an absence of observations.
# handover_confirmed also requires fresh capable teammate presence at this post.
# Flags depend on actual zone/control observations; presence does not imply intent.
