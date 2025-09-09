# **Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware**

## **I. INTRODUCTION**

Fine manipulation tasks involve precise, closed-loop feedback and require high degrees of hand-eye coordination to adjust and re-plan in response to changes in the environment. Examples of such manipulation tasks include opening the lid of a condiment cup or slotting a battery, which involve delicate operations such as pinching, prying, and tearing rather than broad-stroke motions such as picking and placing. Take opening the lid of a condiment cup as an example: the right gripper needs to first tip it over, and nudge it into the opened left gripper. Then the left gripper closes gently and lifts the cup off the table. Next, one of the right fingers approaches the cup from below and pries the lid open. Each of these steps requires high precision, delicate hand-eye coordination, and rich contact. Millimeters of error would lead to task failure.

Existing systems for fine manipulation use expensive robots and high-end sensors for precise state estimation. In this work, we seek to develop a low-cost system for fine manipulation that is, in contrast, accessible and reproducible. However, low-cost hardware is inevitably less precise than high-end platforms, making the sensing and planning challenge more pronounced. One promising direction to resolve this is to incorporate learning into the system. Humans also do not have industrial-grade proprioception, and yet we are able to perform delicate tasks by learning from closed-loop visual feedback and actively compensating for errors. In our system, we therefore train an end-to-end policy that directly maps RGB images from commodity web cameras to the actions. This pixel-to-action formulation is particularly suitable for fine manipulation, because fine manipulation often involves objects with complex physical properties, such that learning the manipulation policy is much simpler than modeling the whole environment.

Training an end-to-end policy, however, presents its own challenges. The performance of the policy depends heavily on the training data distribution, and in the case of fine manipulation, high-quality human demonstrations can provide tremendous value by allowing the system to learn from human dexterity. We thus build a low-cost yet dexterous teleoperation system for data collection, and a novel imitation learning algorithm that learns effectively from the demonstrations.

**Teleoperation system.** We devise a teleoperation setup with two sets of low-cost, off-the-shelf robot arms. They are approximately scaled versions of each other, and we use joint-space mapping for teleoperation. We augment this setup with 3D printed components for easier backdriving, leading to a highly capable teleoperation system within a $20k budget.

**Imitation learning algorithm.** Tasks that require precision and visual feedback present a significant challenge for imitation learning, even with high-quality demonstrations. Small errors in the predicted action can incur large differences in the state, exacerbating the "compounding error" problem. To tackle this, we take inspiration from action chunking, where the policy predicts the target joint positions for the next k timesteps, rather than just one step at a time. This reduces the effective horizon of the task by k-fold, mitigating compounding errors. To further improve the smoothness of the policy, we propose temporal ensembling, which queries the policy more frequently and averages across the overlapping action chunks. We implement action chunking policy with Transformers, an architecture designed for sequence modeling, and train it as a conditional VAE (CVAE) to capture the variability in human data. We name our method Action Chunking with Transformers (ACT).

The key contribution of this paper is a low-cost system for learning fine manipulation, comprising a teleoperation system and a novel imitation learning algorithm. The synergy between these two parts allows learning of 6 fine manipulation skills directly in the real-world, such as opening a translucent condiment cup and slotting a battery with 80-90% success, from only 10 minutes or 50 demonstration trajectories.

## **II. RELATED WORK**

**Imitation learning for robotic manipulation.** Behavioral cloning (BC) is one of the simplest imitation learning algorithms, casting imitation as supervised learning from observations to actions. Many works have sought to improve BC by incorporating history, using different training objectives, and including regularization. Other works emphasize multi-task or few-shot aspects, leveraging language, or exploiting specific task structures. Scaling these algorithms with more data has led to impressive systems that can generalize. In this work, we focus on building a low-cost system capable of performing delicate, fine manipulation tasks.

**Addressing compounding errors.** A major shortcoming of BC is compounding errors. One way to mitigate this is to allow additional on-policy interactions and expert corrections, such as DAgger. However, expert annotation can be time-consuming. One could also inject noise at demonstration collection time to obtain datasets with corrective behavior, but for fine manipulation, such noise injection can lead to task failure. We propose to reduce the effective horizon of tasks through action chunking and then ensemble across overlapping action chunks to produce trajectories that are both accurate and smooth.

**Bimanual manipulation.** Bimanual manipulation has a long history in robotics. Early works tackled it from a classical control perspective. More recently, learning has been incorporated, such as reinforcement learning or imitating human demonstrations. Some works focus on fine-grained tasks like knot untying or threading a needle, while using expensive robots. Our work turns to low-cost hardware and seeks to enable them to perform high-precision, closed-loop tasks. Our teleoperation setup is most similar to a prior system which also uses joint-space mapping, but unlike that system, we do not make use of special encoders, sensors, or machined components.

## **III. ALOHA: A LOW-COST OPEN-SOURCE HARDWARE SYSTEM FOR BIMANUAL TELEOPERATION**

We seek to develop an accessible and high-performance teleoperation system for fine manipulation. We summarize our design considerations into 5 principles: Low-cost, Versatile, User-friendly, Repairable, and Easy-to-build.

These principles led us to build a bimanual parallel-jaw grippers setup with two ViperX 6-DoF robot arms. The robot is modular and simple to repair. The OEM fingers are not versatile enough, so we designed our own 3D printed "see-through" fingers.

We use direct joint-space mapping from a smaller robot, the WidowX ("the leader"), to the larger ViperX ("the follower"). The user teleoperates by backdriving the leader. Joint-space mapping has several benefits over task-space mapping, such as avoiding IK failures near singularities and providing better performance on precise tasks. To improve the teleoperation experience, we designed a 3D-printed "handle and scissor" mechanism and a rubber band load balancing mechanism.

The rest of the setup includes a robot cage and four Logitech C922x webcams. Two are mounted on the wrists of the follower robots, and the other two are mounted on the front and top. Both teleoperation and data recording happen at 50Hz.

With these design considerations, we built the ALOHA system within a $20k USD budget. It enables the teleoperation of precise, contact-rich, and dynamic tasks. We open-source all software and hardware with a detailed tutorial.

## **IV. ACTION CHUNKING WITH TRANSFORMERS**

We develop a novel algorithm, Action Chunking with Transformers (ACT), to leverage the data collected by ALOHA.

To train ACT, we first collect human demonstrations. We record the joint positions of the leader robots as actions. The observations are composed of the current joint positions of follower robots and the image feed from 4 cameras. ACT predicts the sequence of future actions given the current observations. The main challenge is compounding errors.

### **A. Action Chunking and Temporal Ensemble**

To combat compounding errors, we seek to reduce the effective horizon of long trajectories. We are inspired by action chunking. In our implementation, every k steps, the agent receives an observation, generates the next k actions, and executes them in sequence. This implies a k-fold reduction in the effective horizon. Concretely, the policy models πθ​(at:t+k​∣st​) instead of πθ​(at​∣st​).

A naïve implementation of action chunking can be suboptimal. To improve smoothness, we query the policy at every timestep, making different action chunks overlap. We propose a temporal ensemble to combine these predictions with a weighted average using an exponential weighting scheme wi​=exp(−m⋅i).

### **B. Modeling human data**

To handle noisy and multi-modal human demonstrations, we train our action chunking policy as a conditional variational autoencoder (CVAE). The CVAE has an encoder and a decoder (the policy). The encoder predicts the distribution of a style variable z given the current observation and action sequence. The decoder conditions on both z and the current observations to predict the action sequence. At test time, we set z to zero to deterministically decode. The model is trained to maximize the log-likelihood of demonstration action chunks with the standard VAE objective.

### **C. Implementing ACT**

We implement the CVAE encoder and decoder with transformers. The CVAE encoder is a BERT-like transformer encoder. The CVAE decoder uses ResNet image encoders, a transformer encoder, and a transformer decoder. The observation includes 4 RGB images and joint positions for two robot arms (14 DoF total). The action space is the absolute joint positions for two robots. The policy outputs a k×14 tensor. We use L1 loss for reconstruction.

We summarize the training and inference of ACT in Algorithms 1 and 2\. The model has around 80M parameters, and training takes around 5 hours on a single 11G RTX 2080 Ti GPU.

**Algorithm 1: ACT Training**

1. Given: Demo dataset D, chunk size k, weight β.  
2. Initialize encoder qϕ​(z∣at:t+k​,oˉt​)  
3. Initialize decoder πθ​(a^t:t+k​∣ot​,z)  
4. **for** iteration n=1,2,... **do**  
5. Sample ot​,at:t+k​ from D  
6. Sample z from qϕ​(z∣at:t+k​,oˉt​)  
7. Predict a^t:t+k​ from πθ​(a^t:t+k​∣ot​,z)  
8. Lreconst​=MSE(a^t:t+k​,at:t+k​)  
9. Lreg​=DKL​(qϕ​(z∣at:t+k​,oˉt​)∣∣N(0,I))  
10. Update θ,ϕ with ADAM and L=Lreconst​+βLreg​  
11. **end for**

**Algorithm 2: ACT Inference**

1. Given: trained πθ​, episode length T, weight m.  
2. Initialize FIFO buffers B\[0:T\].  
3. **for** timestep t=1,2,...T **do**  
4. Predict a^t:t+k​ with πθ​(a^t:t+k​∣ot​,z=0)  
5. Add a^t:t+k​ to buffers B\[t:t+k\]  
6. Obtain current step actions At​=B\[t\]  
7. Apply at​=∑i​wi​At​\[i\]/∑i​wi​, with wi​=exp(−m⋅i)  
8. **end for**

## **V. EXPERIMENTS**

We evaluate ACT's performance on 2 simulated and 6 real-world fine manipulation tasks.

### **A. Tasks**

All 8 tasks require fine-grained, bimanual manipulation. The real-world tasks are: Slide Ziploc, Slot Battery, Open Cup, Thread Velcro, Prep Tape, and Put On Shoe. The simulated tasks are Transfer Cube and Bimanual Insertion. The initial placement of objects is randomized. The objects used present significant perception challenges (e.g., transparency, low contrast).

### **B. Data Collection**

For all 6 real-world tasks, we collect 50-100 demonstrations using ALOHA teleoperation. Each episode takes 8-14 seconds. For the two simulated tasks, we collect both scripted and human demonstrations. Human demonstrations are inherently stochastic.

### **C. Experiment Results**

We compare ACT with four prior imitation learning methods: BC-ConvMLP, BeT, RT-1, and VINN.

Table I: Success rate (%) for 2 simulated and 2 real-world tasks  
| Task | Touched | Transfer/Lifted | Grasp | Contact/Insert | Open/Place/Insert |  
| :--- | :--- | :--- | :--- | :--- | :--- |  
| Cube Transfer (sim) | | | | | |  
| BC-ConvMLP | 34/13 | 17/11 | 1/10 | | |  
| BeT | 60/16 | 51/13 | 2/7 | | |  
| RT-1 | 44/14 | 33/12 | 2/10 | | |  
| VINN | 13/17 | 9/11 | 3/10 | | |  
| ACT (Ours) | 97/82 | 90/60 | 86/50 | | |  
| Bimanual Insertion (sim) | | | | | |  
| BC-ConvMLP | | | 5/10 | 1/10 | 1/10 |  
| BeT | | | 21/10 | 4/10 | 3/10 |  
| RT-1 | | | 2/10 | 0/10 | 1/10 |  
| VINN | | | 6/10 | 1/10 | 1/10 |  
| ACT (Ours) | | | 93/76 | 90/66 | 32/20 |  
| Slide Ziploc (real) | | | 8 | 0 | 0 |  
| BeT | | | 92 | | 88 |  
| ACT (Ours) | | | 96 | | 96 |  
| Slot Battery (real) | | | 4 | 0 | 0 |  
| BeT | | | 100 | 100 | 96 |  
| ACT (Ours) | | | 100 | 100 | 96 |  
*Note: For simulated tasks, results are \[scripted data | human data\].*

Table II: Success rate (%) for the remaining 4 real-world tasks  
| Task | Tip Over/Lift | Grasp | Open Lid/Insert | Cut/Handover/Hang | Support/Secure |  
| :--- | :--- | :--- | :--- | :--- | :--- |  
| Open Cup (real) | | | | | |  
| BeT | 12 | 0 | 0 | | |  
| ACT (Ours) | 100 | 96 | 84 | | |  
| Thread Velcro (real) | 20 | 40 | 92 | | |  
| BeT | | 24 | 0 | | |  
| ACT (Ours) | | 92 | 20 | | |  
| Prep Tape (real) | | | | 8/0/0 | |  
| BeT | | | | 96/92/72/64 | |  
| ACT (Ours) | | | | 96/92/72/64 | |  
| Put On Shoe (real) | 12 | | 0 | | 0/0 |  
| BeT | 100 | | 92 | | 92/92 |  
| ACT (Ours) | 100 | | 92 | | 92/92 |  
ACT achieves the highest success rate compared to all prior methods, outperforming them by a large margin. We attribute the poor performance of prior methods to compounding errors and non-Markovian behavior in the data.

## **VI. ABLATIONS**

We ablate action chunking, temporal ensembling, and the CVAE training objective.

### **A. Action Chunking and Temporal Ensembling**

We vary the chunk size k. Performance improves drastically from k=1 to k=100, then slightly tapers down. This illustrates that more chunking and a lower effective horizon generally improve performance. We also augment two baselines with action chunking, and they show consistent trends. Temporal ensembling improves our method and BC-ConvMLP, but hurts VINN.

### **B. Training with CVAE**

We compare ACT with and without the CVAE objective. When training on scripted data, the removal of CVAE makes almost no difference. For human data, there is a significant drop in performance without CVAE, from 35.3% to 2%. This illustrates that the CVAE objective is crucial when learning from human demonstrations.

### **C. Is High-Frequency Necessary?**

We conduct a user study comparing teleoperation at 50Hz and 5Hz. Reducing the frequency from 50Hz to 5Hz results in a 62% increase in teleoperation time. This highlights the necessity of high-frequency teleoperation for fine manipulation.

## **VII. LIMITATIONS AND CONCLUSION**

We present a low-cost system for fine manipulation, comprising a teleoperation system ALOHA and a novel imitation learning algorithm ACT. The synergy between these two parts allows us to learn fine manipulation skills directly in the real world. While the system is quite capable, there exist tasks that are beyond its capability, such as buttoning up a dress shirt. Overall, we hope that this low-cost open-source system represents an important step and accessible resource towards advancing fine-grained robotic manipulation.