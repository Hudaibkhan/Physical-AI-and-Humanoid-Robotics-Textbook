/**
 * Custom Navbar Component Types
 *
 * This file registers custom navbar components with Docusaurus.
 * It allows us to add authentication UI to the navbar using a custom component type.
 *
 * Usage in docusaurus.config.js:
 * navbar: {
 *   items: [
 *     { type: 'custom-Auth', position: 'right' }
 *   ]
 * }
 *
 * Reference: specs/007-auth-integration-fix/research.md Section 3
 */

import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
import NavbarAuth from '@site/src/components/NavbarAuth';

export default {
  ...ComponentTypes,
  'custom-Auth': NavbarAuth,
};
