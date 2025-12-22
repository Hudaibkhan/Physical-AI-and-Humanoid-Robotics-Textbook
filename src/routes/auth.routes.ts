import Router from 'koa-router';
import { registerUser, loginUser } from '../auth/api/register.api';
import { getUserInfo } from '../auth/api/user-info.api';

const router = new Router({ prefix: '/api/auth' });

// Registration and login routes
router.post('/register', registerUser);
router.post('/login', loginUser);
router.get('/me', getUserInfo);

export default router;