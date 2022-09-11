from gym.envs.registration import register
from importlib_metadata import entry_points

register(
    id='MultiagentReaction-v0',
    entry_point = 'multiagent_reaction.envs:MultiagentReactionEnv'
)