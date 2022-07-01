Out of the reward functions i have wrote , Following actions gave me fruitful results after training the model.

1. Keep the reward function simple and Keep action space short. 
For Example : - My reward function rewards model if car sticks to the centre line of the lane without deviating , and crossing the border while my
my action space is speed -[0.5 to 2]metres/second and Degrees [0 , 30 or 20] Degrees.
2. After the training model learns to be in lane and complete the track without going offtrack now slowly introduce reward for speed and increase the action space.
3. Writing a complex Reward Function and training directly on it first did not gave me fruitful results in long run.
4. Keeping the reward Function simple and iteratively improving it by cloing adding new functionality to reward function found to be the best approach for me.
