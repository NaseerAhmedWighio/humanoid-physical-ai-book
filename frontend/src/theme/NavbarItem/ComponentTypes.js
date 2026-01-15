import DefaultNavbarItem from '@theme/NavbarItem/DefaultNavbarItem';
import DropdownNavbarItem from '@theme/NavbarItem/DropdownNavbarItem';
import DocSidebarNavbarItem from '@theme/NavbarItem/DocSidebarNavbarItem';
import DocsVersionNavbarItem from '@theme/NavbarItem/DocsVersionNavbarItem';
import DocsVersionDropdownNavbarItem from '@theme/NavbarItem/DocsVersionDropdownNavbarItem';

// ⚠️ Built-in search ko mat override karo
// import SearchNavbarItem from '@theme/NavbarItem/SearchNavbarItem';

import CustomSearchButton from './CustomSearchButton';
import NavbarItemCustomAuthButtons from './NavbarItemCustomAuthButtons';
import NavbarCustomPageControls from './NavbarItemCustomPageControls';

export default {
  // ⭐ Docusaurus built-ins
  default: DefaultNavbarItem,
  dropdown: DropdownNavbarItem,
  docSidebar: DocSidebarNavbarItem,
  docsVersion: DocsVersionNavbarItem,
  docsVersionDropdown: DocsVersionDropdownNavbarItem,

  // ❌ ye disable rehne do — warna error aata hai
  // search: CustomSearchButton,

  // ⭐ Custom navbar items
  'custom-search-button': CustomSearchButton,
  'custom-auth-buttons': NavbarItemCustomAuthButtons,
  'custom-page-controls': NavbarCustomPageControls,
};
