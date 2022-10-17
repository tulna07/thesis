import pickle

HM_EPISODES = 25000

MOVE_PENALTY = 1
WALL_PENALTY = 300
EMPTY_SPACE_PENALTY = 300

GOAL_REWARD = 25

epsilon = 0.9
EPS_DECAY = 0.9998  # Every episode will be epsilon*EPS_DECAY
SHOW_EVERY = 3000  # how often to play through env visually.

LEARNING_RATE = 0.1
DISCOUNT = 0.95

start_q_table = None

if start_q_table is None:
  # initialize the q-table#
  q_table = {}
else:
  with open(start_q_table, "rb") as f:
    q_table = pickle.load(f)