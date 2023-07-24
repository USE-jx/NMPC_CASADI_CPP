# Simple Control: Move to Position $(x_r, y_r, \theta_r)^T$

Consider the current robot state: $\mathbf{x} = (x, y, \theta)^T$ <br>
And the target position: $\mathbf{x}_r = (x_r, y_r, \theta_r)^T$

The error state $\mathbf{x}_e$ as defined:
$$ \mathbf{x}_e = \mathbf{x}_r  - \mathbf{x} = \left[ \begin{matrix} \Delta x \\ \Delta y \\ \Delta \theta\end{matrix} \right] $$

We need to calculate the velocity to reach the $\mathbf{x}_e \to 0$

$$ \mathbf{x}_e = \left[ \begin{matrix} \Delta x \\ \Delta y \\ \Delta \theta\end{matrix} \right] = \left[ \begin{matrix} \cos{\Delta \theta} & -\sin{\Delta \theta} & 0 \\ \sin{\Delta \theta} & \cos{\Delta \theta} & 0 \\ 0 & 0 & 1 \end{matrix} \right] \left[ \begin{matrix} v^*_x \\ v^*_y \\ \omega^* \end{matrix} \right] $$

Therefore:
$$ \left[ \begin{matrix} v^*_x \\ v^*_y \\ \omega^* \end{matrix} \right] = \left[ \begin{matrix} \cos{\Delta \theta} & -\sin{\Delta \theta} & 0 \\ \sin{\Delta \theta} & \cos{\Delta \theta} & 0 \\ 0 & 0 & 1 \end{matrix} \right]^{-1} \left[ \begin{matrix} \Delta x \\ \Delta y \\ \Delta \theta\end{matrix} \right] $$

$$ \mathbf{Q}^{-1} = \left[ \begin{matrix} \cos{\Delta \theta} & \sin{\Delta \theta} & 0 \\ -\sin{\Delta \theta} & \cos{\Delta \theta} & 0 \\ 0 & 0 & 1 \end{matrix} \right] $$